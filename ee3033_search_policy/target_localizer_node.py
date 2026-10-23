"""Estimate a detected target position in the map frame.

The primary estimate associates the target bearing with a fresh 2-D LaserScan.
A configurable ground-plane fallback is available for targets below laser
height. Both methods remain explicit in /target_position_method.
"""

import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from balance_interface.msg import TargetInfo
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, LaserScan
from std_msgs.msg import Bool, Float32, String


class TargetLocalizer(Node):

    def __init__(self):
        super().__init__('target_localizer')

        defaults = {
            'map_frame': 'map',
            'base_frame': 'base_footprint',
            'camera_info_topic': '/camera_info',
            'scan_topic': '/scan',
            'status_topic': '/detector/status',
            'target_info_topic': '/detector/target_info',
            'target_found_topic': '/target_found',
            'position_topic': '/target_position',
            'range_mode': 'lidar_then_ground',
            'laser_yaw_offset_rad': 0.0,
            'camera_yaw_offset_rad': 0.0,
            'camera_height_m': 0.2655,
            'camera_pitch_down_rad': 0.0,
            'scan_window_rad': 0.05236,
            'scan_timeout_s': 0.5,
            'minimum_range_m': 0.15,
            'maximum_range_m': 4.0,
            'minimum_ground_angle_rad': 0.0873,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value

        self.map_frame = str(p('map_frame'))
        self.base_frame = str(p('base_frame'))
        self.range_mode = str(p('range_mode'))
        self.laser_yaw_offset = float(p('laser_yaw_offset_rad'))
        self.camera_yaw_offset = float(p('camera_yaw_offset_rad'))
        self.camera_height = float(p('camera_height_m'))
        self.camera_pitch_down = float(p('camera_pitch_down_rad'))
        self.scan_window = float(p('scan_window_rad'))
        self.scan_timeout = float(p('scan_timeout_s'))
        self.minimum_range = float(p('minimum_range_m'))
        self.maximum_range = float(p('maximum_range_m'))
        self.minimum_ground_angle = float(p('minimum_ground_angle_rad'))

        if self.range_mode not in (
            'lidar_only', 'ground_only', 'lidar_then_ground'
        ):
            raise ValueError('Unsupported range_mode')
        if not 0.0 < self.minimum_range < self.maximum_range:
            raise ValueError('Invalid range limits')

        self.camera_info = None
        self.scan = None
        self.scan_receive_time = None
        self.detector_status = 'UNKNOWN'
        self.latest_position = None
        self.latest_confidence = 0.0
        self.latest_method = ''

        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.position_pub = self.create_publisher(
            PointStamped, str(p('position_topic')), latched
        )
        self.candidate_pub = self.create_publisher(
            PointStamped, '/target_position_candidate', 10
        )
        self.confidence_pub = self.create_publisher(
            Float32, '/target_confidence', 10
        )
        self.method_pub = self.create_publisher(
            String, '/target_position_method', 10
        )

        self.create_subscription(
            CameraInfo,
            str(p('camera_info_topic')),
            self._camera_info_callback,
            sensor_qos,
        )
        self.create_subscription(
            LaserScan,
            str(p('scan_topic')),
            self._scan_callback,
            sensor_qos,
        )
        self.create_subscription(
            String,
            str(p('status_topic')),
            self._status_callback,
            10,
        )
        self.create_subscription(
            TargetInfo,
            str(p('target_info_topic')),
            self._target_callback,
            sensor_qos,
        )
        self.create_subscription(
            Bool,
            str(p('target_found_topic')),
            self._found_callback,
            10,
        )
        self.get_logger().info(
            f'Target localizer ready: range_mode={self.range_mode}'
        )

    def _camera_info_callback(self, msg):
        if len(msg.k) == 9 and msg.k[0] > 0.0 and msg.k[4] > 0.0:
            self.camera_info = msg

    def _scan_callback(self, msg):
        self.scan = msg
        self.scan_receive_time = time.monotonic()

    def _status_callback(self, msg):
        self.detector_status = msg.data.strip().upper()

    def _lidar_range(self, bearing):
        if (
            self.scan is None
            or self.scan_receive_time is None
            or time.monotonic() - self.scan_receive_time > self.scan_timeout
        ):
            return None
        scan = self.scan
        if scan.angle_increment == 0.0 or not scan.ranges:
            return None
        target_angle = bearing - self.laser_yaw_offset
        angles = (
            scan.angle_min
            + np.arange(len(scan.ranges), dtype=np.float64)
            * scan.angle_increment
        )
        delta = np.arctan2(
            np.sin(angles - target_angle),
            np.cos(angles - target_angle),
        )
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        valid = (
            np.abs(delta) <= self.scan_window
        ) & np.isfinite(ranges) & (
            ranges >= max(self.minimum_range, scan.range_min)
        ) & (
            ranges <= min(self.maximum_range, scan.range_max)
        )
        if not np.any(valid):
            return None
        return float(np.median(ranges[valid]))

    def _ground_range(self, u, v):
        info = self.camera_info
        if info is None:
            return None
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        down_angle = (
            self.camera_pitch_down
            + math.atan2(v - cy, fy)
        )
        if down_angle < self.minimum_ground_angle:
            return None
        horizontal = self.camera_height / math.tan(down_angle)
        if not self.minimum_range <= horizontal <= self.maximum_range:
            return None
        return float(horizontal)

    def _map_point(self, bearing, distance):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.1),
            )
        except TransformException as exc:
            self.get_logger().warning(
                f'Cannot transform target into map: {exc}',
                throttle_duration_sec=2.0,
            )
            return None

        q = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        local_x = distance * math.cos(bearing)
        local_y = distance * math.sin(bearing)
        point = PointStamped()
        point.header.stamp = self.get_clock().now().to_msg()
        point.header.frame_id = self.map_frame
        point.point.x = (
            transform.transform.translation.x
            + math.cos(yaw) * local_x
            - math.sin(yaw) * local_y
        )
        point.point.y = (
            transform.transform.translation.y
            + math.sin(yaw) * local_x
            + math.cos(yaw) * local_y
        )
        point.point.z = 0.0
        return point

    def _target_callback(self, msg):
        if (
            self.detector_status != 'TRACKING'
            or not msg.detected
            or msg.width <= 0
            or msg.height <= 0
            or self.camera_info is None
        ):
            return

        info = self.camera_info
        bearing = (
            self.camera_yaw_offset
            + math.atan2(msg.center_x - info.k[2], info.k[0])
        )
        distance = None
        method = ''

        if self.range_mode in ('lidar_only', 'lidar_then_ground'):
            distance = self._lidar_range(bearing)
            if distance is not None:
                method = 'lidar_association'

        if (
            distance is None
            and self.range_mode in ('ground_only', 'lidar_then_ground')
        ):
            distance = self._ground_range(msg.center_x, msg.center_y)
            if distance is not None:
                method = 'ground_plane'

        if distance is None:
            return

        point = self._map_point(bearing, distance)
        if point is None:
            return

        self.latest_position = point
        self.latest_confidence = float(msg.similarity)
        self.latest_method = method
        self.candidate_pub.publish(point)
        confidence = Float32()
        confidence.data = float(msg.similarity)
        self.confidence_pub.publish(confidence)
        source = String()
        source.data = method
        self.method_pub.publish(source)

    def _found_callback(self, msg):
        if msg.data and self.latest_position is not None:
            position = self.latest_position
            position.header.stamp = self.get_clock().now().to_msg()
            self.position_pub.publish(position)
            confidence = Float32()
            confidence.data = self.latest_confidence
            self.confidence_pub.publish(confidence)
            source = String()
            source.data = self.latest_method
            self.method_pub.publish(source)


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
