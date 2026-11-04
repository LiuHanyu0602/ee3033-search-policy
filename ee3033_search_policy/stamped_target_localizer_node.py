"""Typed localization candidate producer; no nav, no legacy /target_position."""
import json
from pathlib import Path
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
from ee3033_search_policy_interfaces.msg import TargetObservation, TargetEstimate
from .yolo_bearing import RawCameraCalibration
from .stamped_localization import LocalizationMemory, map_ground_point, rotation_from_quaternion


class StampedTargetLocalizer(Node):
    def __init__(self):
        super().__init__('stamped_target_localizer')
        defaults = {'calibration_file': '', 'geometry_verified': False,
                    'assume_ground_contact': False, 'ground_z_m': 0.,
                    'map_frame': 'map', 'camera_info_topic': '/camera_info',
                    'maximum_age_s': .5, 'minimum_range_m': .15, 'maximum_range_m': 4.}
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        p = lambda k: self.get_parameter(k).value
        self.calibration = RawCameraCalibration.load(Path(str(p('calibration_file'))).expanduser())
        self.verified, self.ground_enabled = bool(p('geometry_verified')), bool(p('assume_ground_contact'))
        self.ground_z, self.frame = float(p('ground_z_m')), str(p('map_frame'))
        self.age, self.rmin, self.rmax = map(float, (p('maximum_age_s'), p('minimum_range_m'), p('maximum_range_m')))
        if not all(math.isfinite(x) for x in (self.ground_z, self.age, self.rmin, self.rmax)) or not 0 < self.rmin < self.rmax or self.age <= 0:
            raise ValueError('Invalid geometry limits')
        self.memory = LocalizationMemory(range_min=self.rmin, range_max=self.rmax)
        self.camera = None
        self.last_clock = None
        self.last_calibration_signature = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.output = self.create_publisher(TargetEstimate, '/target/estimate_candidate', 10)
        self.status = self.create_publisher(String, '/target/localization_status', latched)
        self.create_subscription(String, '/target/configuration', self._configuration, latched)
        self.create_subscription(CameraInfo, str(p('camera_info_topic')), self._camera,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(TargetObservation, '/target/observation', self._observation, 10)
        self.status.publish(String(data='WAITING_FOR_VERIFIED_GEOMETRY'))

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
            u, v = msg.center_x, msg.center_y+msg.height/2
            if (msg.center_x-msg.width/2 < 0 or msg.center_x+msg.width/2 >= msg.image_width or
                    msg.center_y-msg.height/2 < 0 or v >= msg.image_height):
                raise ValueError('BBOX_CLIPPED_OR_OUTSIDE_IMAGE')
            # Zero timeout: never block ROS callbacks waiting for TF. No latest-TF fallback.
            tf = self.buffer.lookup_transform(self.frame, msg.header.frame_id,
                                               Time.from_msg(msg.header.stamp))
            q, t = tf.transform.rotation, tf.transform.translation
            rotation = rotation_from_quaternion(q.x, q.y, q.z, q.w)
            origin = np.array([t.x, t.y, t.z])
            def ray(x, y):
                angle = self.calibration.bearing(x, y)  # checks inverse-distortion convergence
                c = self.calibration.camera
                from .target_localization_core import undistort_normalized
                xy = undistort_normalized((x-c.cx)/c.fx, (y-c.cy)/c.fy,
                                          c.distortion, iterations=100)
                r = np.array([math.tan(angle), xy[1], 1.])
                return rotation @ (r/np.linalg.norm(r))
            centre = ray(msg.center_x, msg.center_y)
            ground = map_ground_point(origin, ray(u, v), self.ground_z,
                                      self.rmin, self.rmax) if self.ground_enabled else None
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            self.memory.add(stamp, origin, centre, ground, msg.confidence, msg.tracking_id)
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
