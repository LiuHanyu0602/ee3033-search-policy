"""Robust target localization for the Balance RGB camera and 2-D LiDAR.

Pipeline: undistorted bbox contact ray -> ground-plane estimate -> gated LiDAR
association -> map-frame point -> multi-view RANSAC/robust fusion.
Hardware-specific IMU and extrinsic corrections are opt-in parameters.
"""

from collections import deque
import math
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from balance_interface.msg import TargetInfo
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import CameraInfo, Imu, LaserScan
from std_msgs.msg import Bool, Float32, String, UInt32
from std_srvs.srv import Trigger

from .target_localization_core import (
    CameraModel,
    bbox_contact_pixel,
    bearing_direction,
    choose_gated_range,
    pixel_to_camera_ray,
    ransac_triangulate,
    robust_lidar_range,
    robust_weighted_fusion,
)


def quaternion_to_rpy(q):
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return roll, pitch, math.atan2(siny, cosy)


class TargetLocalizer(Node):

    def __init__(self):
        super().__init__('target_localizer')
        defaults = {
            'map_frame': 'map', 'base_frame': 'base_footprint',
            'camera_info_topic': '/camera_info', 'scan_topic': '/scan',
            'imu_topic': '/imu', 'status_topic': '/detector/status',
            'target_info_topic': '/detector/target_info',
            'target_found_topic': '/target_found',
            'position_topic': '/target_position',
            'range_mode': 'lidar_then_ground',
            'laser_yaw_offset_rad': 0.0, 'camera_yaw_offset_rad': 0.0,
            'camera_height_m': 0.2655, 'camera_pitch_down_rad': 0.0,
            'use_imu_pitch': False, 'imu_pitch_sign': 1.0,
            'imu_pitch_offset_rad': 0.0, 'imu_timeout_s': 0.25,
            'scan_window_rad': 0.05236, 'scan_timeout_s': 0.5,
            'minimum_range_m': 0.15, 'maximum_range_m': 4.0,
            'minimum_ground_angle_rad': 0.0873,
            'lidar_ground_gate_m': 0.35, 'minimum_lidar_points': 2,
            'max_observations': 30, 'minimum_multiview_observations': 3,
            'minimum_baseline_m': 0.25, 'ransac_threshold_m': 0.20,
            'fusion_gate_m': 0.35,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value

        self.map_frame, self.base_frame = str(p('map_frame')), str(p('base_frame'))
        self.range_mode = str(p('range_mode'))
        self.laser_yaw_offset = float(p('laser_yaw_offset_rad'))
        self.camera_yaw_offset = float(p('camera_yaw_offset_rad'))
        self.camera_height = float(p('camera_height_m'))
        self.camera_pitch_down = float(p('camera_pitch_down_rad'))
        self.use_imu_pitch = bool(p('use_imu_pitch'))
        self.imu_pitch_sign = float(p('imu_pitch_sign'))
        self.imu_pitch_offset = float(p('imu_pitch_offset_rad'))
        self.imu_timeout = float(p('imu_timeout_s'))
        self.scan_window = float(p('scan_window_rad'))
        self.scan_timeout = float(p('scan_timeout_s'))
        self.minimum_range = float(p('minimum_range_m'))
        self.maximum_range = float(p('maximum_range_m'))
        self.minimum_ground_angle = float(p('minimum_ground_angle_rad'))
        self.lidar_ground_gate = float(p('lidar_ground_gate_m'))
        self.minimum_lidar_points = int(p('minimum_lidar_points'))
        self.minimum_multiview = int(p('minimum_multiview_observations'))
        self.minimum_baseline = float(p('minimum_baseline_m'))
        self.ransac_threshold = float(p('ransac_threshold_m'))
        self.fusion_gate = float(p('fusion_gate_m'))

        self.camera_info = self.scan = None
        self.scan_receive_time = self.imu_receive_time = None
        self.imu_pitch = 0.0
        self.detector_status = 'UNKNOWN'
        self.latest_position = None
        self.latest_confidence = 0.0
        self.latest_method = 'unavailable'
        self.latest_covariance = np.eye(2) * self.fusion_gate ** 2
        maximum = int(p('max_observations'))
        self.origins = deque(maxlen=maximum)
        self.directions = deque(maxlen=maximum)
        self.point_estimates = deque(maxlen=maximum)
        self.weights = deque(maxlen=maximum)

        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.position_pub = self.create_publisher(PointStamped, str(p('position_topic')), latched)
        self.candidate_pub = self.create_publisher(PointStamped, '/target_position_candidate', 10)
        self.covariance_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/target_position_with_covariance', latched)
        self.confidence_pub = self.create_publisher(Float32, '/target_confidence', 10)
        self.method_pub = self.create_publisher(String, '/target_position_method', 10)
        self.count_pub = self.create_publisher(UInt32, '/target_observation_count', 10)

        self.create_subscription(CameraInfo, str(p('camera_info_topic')),
                                 self._camera_info_callback, sensor_qos)
        self.create_subscription(LaserScan, str(p('scan_topic')),
                                 self._scan_callback, sensor_qos)
        if self.use_imu_pitch:
            self.create_subscription(Imu, str(p('imu_topic')), self._imu_callback, sensor_qos)
        self.create_subscription(String, str(p('status_topic')), self._status_callback, 10)
        self.create_subscription(TargetInfo, str(p('target_info_topic')),
                                 self._target_callback, sensor_qos)
        self.create_subscription(Bool, str(p('target_found_topic')), self._found_callback, 10)
        self.reset_service = self.create_service(
            Trigger, '/target_localizer/reset', self._reset_callback)
        self.get_logger().info(
            f'Robust target localizer ready: range_mode={self.range_mode}, '
            f'imu_pitch={self.use_imu_pitch}, multiview>={self.minimum_multiview}')

    def _camera_info_callback(self, msg):
        if len(msg.k) == 9 and msg.k[0] > 0.0 and msg.k[4] > 0.0:
            self.camera_info = msg

    def _scan_callback(self, msg):
        self.scan, self.scan_receive_time = msg, time.monotonic()

    def _imu_callback(self, msg):
        _, self.imu_pitch, _ = quaternion_to_rpy(msg.orientation)
        self.imu_receive_time = time.monotonic()

    def _status_callback(self, msg):
        self.detector_status = msg.data.strip().upper()

    def _camera_model(self):
        info = self.camera_info
        return CameraModel(info.k[0], info.k[4], info.k[2], info.k[5], tuple(info.d))

    def _ground_range(self, ray):
        dynamic_pitch = 0.0
        if self.use_imu_pitch:
            if self.imu_receive_time is None or time.monotonic() - self.imu_receive_time > self.imu_timeout:
                return None
            dynamic_pitch = self.imu_pitch_sign * self.imu_pitch + self.imu_pitch_offset
        down_angle = self.camera_pitch_down + dynamic_pitch + math.atan2(ray[1], ray[2])
        if down_angle < self.minimum_ground_angle:
            return None
        distance = self.camera_height / math.tan(down_angle)
        return float(distance) if self.minimum_range <= distance <= self.maximum_range else None

    def _lidar_range(self, bearing):
        if self.scan is None or self.scan_receive_time is None or time.monotonic() - self.scan_receive_time > self.scan_timeout:
            return None
        scan = self.scan
        angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
        return robust_lidar_range(
            scan.ranges, angles, bearing - self.laser_yaw_offset,
            self.scan_window, max(self.minimum_range, scan.range_min),
            min(self.maximum_range, scan.range_max),
            minimum_points=self.minimum_lidar_points)

    def _robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time(), timeout=Duration(seconds=0.1))
        except TransformException as exc:
            self.get_logger().warning(f'Cannot localize target: {exc}', throttle_duration_sec=2.0)
            return None
        q = transform.transform.rotation
        _, _, yaw = quaternion_to_rpy(q)
        return np.array([transform.transform.translation.x,
                         transform.transform.translation.y]), yaw

    def _point_message(self, xy):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.point.x, msg.point.y, msg.point.z = float(xy[0]), float(xy[1]), 0.0
        return msg

    def _record_observation(self, origin, direction, point, weight):
        # Near-identical frames add no triangulation baseline; retain the stronger one.
        if self.origins and np.linalg.norm(origin - self.origins[-1]) < 0.03:
            if weight <= self.weights[-1]:
                return
            self.origins.pop(); self.directions.pop(); self.point_estimates.pop(); self.weights.pop()
        self.origins.append(origin); self.directions.append(direction)
        self.point_estimates.append(point); self.weights.append(weight)

    def _best_estimate(self):
        if not self.point_estimates:
            return None, None, 'unavailable'
        points = np.asarray(self.point_estimates)
        weights = np.asarray(self.weights)
        fused, covariance, _ = robust_weighted_fusion(points, weights, self.fusion_gate)
        method = 'robust_range_fusion'
        origins = np.asarray(self.origins)
        if len(origins) >= self.minimum_multiview:
            baseline = np.max(np.linalg.norm(origins[:, None, :] - origins[None, :, :], axis=2))
            if baseline >= self.minimum_baseline:
                triangulated, inliers = ransac_triangulate(
                    origins, np.asarray(self.directions), weights,
                    threshold_m=self.ransac_threshold)
                if triangulated is not None:
                    # Reject intersections behind most cameras.
                    forward = np.sum((triangulated - origins[inliers]) *
                                     np.asarray(self.directions)[inliers], axis=1)
                    if np.all(forward > 0.0):
                        combined = np.vstack([points, triangulated])
                        combined_weights = np.append(weights, max(np.sum(weights[inliers]), 1.0))
                        fused, covariance, _ = robust_weighted_fusion(
                            combined, combined_weights, max(self.fusion_gate, self.ransac_threshold))
                        method = 'multiview_ransac_fused'
        return fused, covariance, method

    def _publish_diagnostics(self, point, confidence, method):
        self.candidate_pub.publish(point)
        c = Float32(); c.data = float(confidence); self.confidence_pub.publish(c)
        m = String(); m.data = method; self.method_pub.publish(m)
        n = UInt32(); n.data = len(self.origins); self.count_pub.publish(n)

    def _target_callback(self, msg):
        if (self.detector_status != 'TRACKING' or not msg.detected or
                msg.width <= 0 or msg.height <= 0 or self.camera_info is None):
            return
        camera = self._camera_model()
        u, v = bbox_contact_pixel(msg.center_x, msg.center_y, msg.width, msg.height,
                                  self.camera_info.height)
        ray = pixel_to_camera_ray(u, v, camera)
        image_bearing = math.atan2(ray[0], ray[2])
        bearing_base = self.camera_yaw_offset + image_bearing
        ground = self._ground_range(ray)
        lidar = self._lidar_range(bearing_base) if self.range_mode != 'ground_only' else None
        if self.range_mode == 'lidar_only':
            distance, source = (lidar, 'lidar_ungated') if lidar is not None else (None, 'unavailable')
        else:
            distance, source = choose_gated_range(
                ground, lidar, self.lidar_ground_gate, prefer_lidar=True)
        if distance is None:
            return
        pose = self._robot_pose()
        if pose is None:
            return
        origin, robot_yaw = pose
        direction = bearing_direction(robot_yaw, self.camera_yaw_offset, image_bearing)
        point_xy = origin + float(distance) * direction
        image_area = max(1.0, float(self.camera_info.width * self.camera_info.height))
        bbox_fraction = min(1.0, float(msg.width * msg.height) / image_area)
        weight = max(1.0e-3, float(msg.similarity) ** 2 * math.sqrt(bbox_fraction))
        self._record_observation(origin, direction, point_xy, weight)
        estimate, covariance, method = self._best_estimate()
        if estimate is None:
            return
        point = self._point_message(estimate)
        self.latest_position, self.latest_covariance = point, covariance
        self.latest_confidence = float(msg.similarity)
        self.latest_method = f'{method}:{source}'
        self._publish_diagnostics(point, msg.similarity, self.latest_method)

    def _found_callback(self, msg):
        if not msg.data or self.latest_position is None:
            return
        self.latest_position.header.stamp = self.get_clock().now().to_msg()
        self.position_pub.publish(self.latest_position)
        pose = PoseWithCovarianceStamped()
        pose.header = self.latest_position.header
        pose.pose.pose.position = self.latest_position.point
        pose.pose.pose.orientation.w = 1.0
        covariance = np.zeros((6, 6))
        covariance[:2, :2] = self.latest_covariance
        covariance[2:, 2:] = np.eye(4) * 1.0e6
        pose.pose.covariance = covariance.reshape(-1).tolist()
        self.covariance_pub.publish(pose)
        self._publish_diagnostics(
            self.latest_position, self.latest_confidence, self.latest_method)

    def _reset_callback(self, request, response):
        del request
        self.origins.clear(); self.directions.clear(); self.point_estimates.clear(); self.weights.clear()
        self.latest_position = None
        self.latest_method = 'unavailable'
        response.success = True
        response.message = 'Target localization observations reset'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TargetLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
