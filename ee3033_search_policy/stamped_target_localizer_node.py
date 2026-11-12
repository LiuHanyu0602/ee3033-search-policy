"""Typed localization candidate producer; no nav, no legacy /target_position."""
from collections import deque
import json
from pathlib import Path
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import CameraInfo, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
from ee3033_search_policy_interfaces.msg import TargetObservation, TargetEstimate
from .yolo_bearing import RawCameraCalibration
from .stamped_localization import (
    LocalizationMemory, bbox_requires_lidar, choose_gated_point, map_ground_point,
    rotation_from_quaternion,
)
from .target_localization_core import (
    nearest_supported_lidar_cluster, robust_lidar_range, undistort_normalized,
)


class StampedTargetLocalizer(Node):
    def __init__(self):
        super().__init__('stamped_target_localizer')
        defaults = {'calibration_file': '', 'geometry_verified': False,
                    'assume_ground_contact': False, 'ground_z_m': 0.,
                    'map_frame': 'map', 'camera_info_topic': '/camera_info',
                    'scan_topic': '/scan', 'range_mode': 'lidar_then_ground',
                    'maximum_age_s': .5, 'minimum_range_m': .15, 'maximum_range_m': 4.,
                    'scan_max_age_s': .75, 'scan_sync_tolerance_s': .12,
                    'scan_window_rad': .05236, 'minimum_lidar_points': 2,
                    'lidar_ground_gate_m': .35, 'laser_yaw_offset_rad': 0.,
                    'scan_buffer_size': 50, 'minimum_range_observations': 5}
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        p = lambda k: self.get_parameter(k).value
        self.calibration = RawCameraCalibration.load(Path(str(p('calibration_file'))).expanduser())
        self.verified, self.ground_enabled = bool(p('geometry_verified')), bool(p('assume_ground_contact'))
        self.ground_z, self.frame = float(p('ground_z_m')), str(p('map_frame'))
        self.age, self.rmin, self.rmax = map(float, (p('maximum_age_s'), p('minimum_range_m'), p('maximum_range_m')))
        self.range_mode = str(p('range_mode'))
        self.scan_age = float(p('scan_max_age_s'))
        self.scan_sync = float(p('scan_sync_tolerance_s'))
        self.scan_window = float(p('scan_window_rad'))
        self.minimum_lidar_points = int(p('minimum_lidar_points'))
        self.lidar_ground_gate = float(p('lidar_ground_gate_m'))
        self.laser_yaw_offset = float(p('laser_yaw_offset_rad'))
        scan_buffer_size = int(p('scan_buffer_size'))
        minimum_range_observations = int(p('minimum_range_observations'))
        if (not all(math.isfinite(x) for x in
                    (self.ground_z, self.age, self.rmin, self.rmax,
                     self.scan_age, self.scan_sync, self.scan_window,
                     self.lidar_ground_gate, self.laser_yaw_offset)) or
                not 0 < self.rmin < self.rmax or self.age <= 0 or
                min(self.scan_age, self.scan_sync, self.scan_window,
                    self.lidar_ground_gate) <= 0 or
                self.range_mode not in ('ground_only', 'lidar_only', 'lidar_then_ground',
                                        'nearest_lidar_then_ground') or
                self.minimum_lidar_points < 1 or scan_buffer_size < 2 or
                minimum_range_observations < 1):
            raise ValueError('Invalid geometry limits')
        self.memory = LocalizationMemory(
            range_min=self.rmin, range_max=self.rmax,
            minimum_range_observations=minimum_range_observations)
        self.scans = deque(maxlen=scan_buffer_size)
        self.camera = None
        self.last_clock = None
        self.last_calibration_signature = None
        self.last_reset_token = ''
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        realtime = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.output = self.create_publisher(
            TargetEstimate, '/target/estimate_candidate', realtime)
        self.status = self.create_publisher(String, '/target/localization_status', latched)
        self.create_subscription(String, '/target/configuration', self._configuration, latched)
        self.create_subscription(String, '/target/localization_reset', self._reset, latched)
        self.create_subscription(
            CameraInfo, str(p('camera_info_topic')), self._camera, realtime)
        self.create_subscription(
            LaserScan, str(p('scan_topic')), self._scan, realtime)
        self.create_subscription(
            TargetObservation, '/target/observation', self._observation, realtime)
        self.status.publish(String(data='WAITING_FOR_VERIFIED_GEOMETRY'))

    def _reset(self, msg):
        """Clear fusion history after confirmation or substantial robot motion."""
        try:
            d = json.loads(msg.data)
            token = str(d['token'])
            if (not token or token == self.last_reset_token or
                    str(d['episode_id']) != self.memory.episode):
                return
            self.last_reset_token = token
            self.memory.configure(self.memory.episode, self.memory.class_id,
                                  self.memory.name)
            self.status.publish(String(data='LOCALIZATION_MEMORY_RESET_'+
                                       str(d.get('reason','UNSPECIFIED'))))
        except (ValueError, TypeError, KeyError):
            self.status.publish(String(data='INVALID_LOCALIZATION_RESET'))

    def _configuration(self, msg):
        try:
            c = json.loads(msg.data)
            key = (str(c['episode_id']), int(c['class_id']), str(c['class_name'])) if c.get('configured') else ('', -1, '')
            if not key[0] or key != (self.memory.episode, self.memory.class_id, self.memory.name):
                self.memory.configure(*key)
        except (ValueError, TypeError, KeyError):
            self.memory.configure('', -1, '')

    def _camera(self, msg):
        try:
            self.calibration.validate_camera_info(msg)
            signature = (msg.header.frame_id, msg.width, msg.height, tuple(msg.k), tuple(msg.d))
            if self.last_calibration_signature is not None and signature != self.last_calibration_signature:
                self.memory.configure('', -1, '')
            self.last_calibration_signature, self.camera = signature, msg
        except ValueError:
            self.camera = None
            self.memory.configure('', -1, '')

    def _scan(self, msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if (math.isfinite(stamp) and stamp > 0 and msg.header.frame_id and
                len(msg.ranges) > 0 and math.isfinite(msg.angle_min) and
                math.isfinite(msg.angle_increment) and msg.angle_increment != 0):
            self.scans.append((stamp, msg))

    def _matched_scan(self, image_stamp, now):
        candidates = [(abs(stamp-image_stamp), stamp, scan)
                      for stamp, scan in self.scans
                      if 0 <= now-stamp <= self.scan_age]
        if not candidates:
            return None
        delta, _, scan = min(candidates, key=lambda row: row[0])
        return scan if delta <= self.scan_sync else None

    def _lidar_point(self, image_stamp, now, map_direction, edge_directions=None):
        if self.range_mode == 'ground_only':
            return None
        scan = self._matched_scan(image_stamp, now)
        if scan is None:
            return None
        try:
            transform = self.buffer.lookup_transform(
                self.frame, scan.header.frame_id, Time.from_msg(scan.header.stamp))
        except TransformException:
            return None
        q, t = transform.transform.rotation, transform.transform.translation
        rotation = rotation_from_quaternion(q.x, q.y, q.z, q.w)
        def scan_bearing(direction):
            transformed = rotation.T @ np.asarray(direction, dtype=float)
            if np.linalg.norm(transformed[:2]) < 1e-9:
                raise ValueError('INVALID_SCAN_DIRECTION')
            return (math.atan2(transformed[1], transformed[0])-
                    self.laser_yaw_offset)
        try:
            bearing = scan_bearing(map_direction)
        except ValueError:
            return None
        angles = scan.angle_min+np.arange(len(scan.ranges))*scan.angle_increment
        scan_min = max(self.rmin, float(scan.range_min))
        scan_max = min(self.rmax, float(scan.range_max))
        if not math.isfinite(scan_min) or not math.isfinite(scan_max) or scan_min >= scan_max:
            return None
        if (self.range_mode == 'nearest_lidar_then_ground' and
                edge_directions is not None):
            try:
                edge_bearings = [scan_bearing(direction)
                                 for direction in edge_directions]
                selected = nearest_supported_lidar_cluster(
                    scan.ranges, angles, bearing, edge_bearings,
                    self.scan_window, scan_min, scan_max,
                    minimum_points=self.minimum_lidar_points)
            except ValueError:
                return None
            if selected is None:
                return None
            distance, selected_angle = selected
        else:
            distance = robust_lidar_range(
                scan.ranges, angles, bearing, self.scan_window,
                scan_min, scan_max,
                minimum_points=self.minimum_lidar_points)
            if distance is None:
                return None
            selected_angle = bearing
        if distance is None:
            return None
        scan_point = np.array([
            distance*math.cos(selected_angle),
            distance*math.sin(selected_angle), 0.])
        map_point = np.array([t.x, t.y, t.z])+rotation@scan_point
        return map_point[:2] if np.all(np.isfinite(map_point)) else None

    def _observation(self, msg):
        now = self.get_clock().now().nanoseconds*1e-9
        if self.last_clock is not None and now < self.last_clock:
            self.memory.configure('', -1, '')
        self.last_clock = now
        result = TargetEstimate()
        result.header.stamp, result.header.frame_id = msg.header.stamp, self.frame
        result.episode_id, result.class_id, result.class_name = msg.episode_id, msg.class_id, msg.class_name
        try:
            if not self.memory.accept(msg, now, self.age):
                raise ValueError('STALE_OR_WRONG_EPISODE')
            if not self.verified or self.camera is None:
                raise ValueError('GEOMETRY_UNVERIFIED')
            if (msg.header.frame_id != self.camera.header.frame_id or
                    (msg.image_width, msg.image_height) != (self.camera.width, self.camera.height)):
                raise ValueError('CAMERA_FRAME_OR_SIZE_MISMATCH')
            values = (msg.center_x, msg.center_y, msg.width, msg.height, msg.confidence)
            if not all(math.isfinite(v) for v in values) or not 0 < msg.confidence <= 1 or min(msg.width, msg.height) <= 0:
                raise ValueError('INVALID_OBSERVATION')
            vertical_boundary = bbox_requires_lidar(
                msg.center_x, msg.center_y, msg.width, msg.height,
                msg.image_width, msg.image_height)
            y2 = msg.center_y+msg.height/2
            # Zero timeout: never block ROS callbacks waiting for TF. No latest-TF fallback.
            tf = self.buffer.lookup_transform(self.frame, msg.header.frame_id,
                                               Time.from_msg(msg.header.stamp))
            q, t = tf.transform.rotation, tf.transform.translation
            rotation = rotation_from_quaternion(q.x, q.y, q.z, q.w)
            origin = np.array([t.x, t.y, t.z])
            def ray(x, y):
                angle = self.calibration.bearing(x, y)  # checks inverse-distortion convergence
                c = self.calibration.camera
                xy = undistort_normalized((x-c.cx)/c.fx, (y-c.cy)/c.fy,
                                          c.distortion, iterations=100)
                r = np.array([math.tan(angle), xy[1], 1.])
                return rotation @ (r/np.linalg.norm(r))
            centre = ray(msg.center_x, msg.center_y)
            bbox_edges = (
                ray(msg.center_x-msg.width/2, msg.center_y),
                ray(msg.center_x+msg.width/2, msg.center_y),
            )
            ground = None
            if self.ground_enabled and not vertical_boundary:
                ground = map_ground_point(
                    origin, ray(msg.center_x, y2), self.ground_z,
                    self.rmin, self.rmax)
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            lidar = self._lidar_point(
                stamp, now, centre, edge_directions=bbox_edges)
            if vertical_boundary:
                range_point, source = choose_gated_point(
                    None, lidar, self.lidar_ground_gate, 'lidar_only')
                if range_point is None:
                    raise ValueError('BBOX_VERTICAL_BOUNDARY_REQUIRES_LIDAR')
                source = 'lidar_vertical_bbox_boundary'
            else:
                range_point, source = choose_gated_point(
                    ground, lidar, self.lidar_ground_gate, self.range_mode)
            self.memory.add(stamp, origin, centre, range_point, msg.confidence,
                            msg.tracking_id, source)
            point, covariance, method = self.memory.estimate()
            if point is None:
                raise ValueError(method)
            result.position.x, result.position.y = map(float, point)
            result.position.z = self.ground_z
            result.covariance_xy = covariance.reshape(-1).tolist()
            result.quality = float(msg.confidence)  # heuristic only, not calibrated spatial probability
            result.method = method
            result.valid = bool(msg.confirmed)
            result.reason = 'CANDIDATE_ONLY_STOP_NOT_VERIFIED' if result.valid else 'DETECTION_NOT_CONFIRMED'
        except (ValueError, TypeError, TransformException) as exc:
            result.valid, result.reason = False, str(exc)
        self.output.publish(result)
        self.status.publish(String(data=result.reason))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = StampedTargetLocalizer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
