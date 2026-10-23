"""Convert the Balance detector output into a latched /target_found signal.

A target is confirmed only when fresh detector status is TRACKING and several
consecutive TargetInfo messages pass the similarity and bounding-box checks.
BUFFERING predictions, LOST, SEARCHING, IDLE, stale status, and invalid numeric
values reset the consecutive-frame counter.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from balance_interface.msg import TargetInfo
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class TargetChecker(Node):

    def __init__(self):
        super().__init__('target_checker')

        # Detector interfaces measured on the Balance Raspberry Pi.
        self.declare_parameter('status_topic', '/detector/status')
        self.declare_parameter('target_info_topic', '/detector/target_info')
        self.declare_parameter('target_found_topic', '/target_found')

        # Empirical first-pass values from the physical detector test.
        self.declare_parameter('similarity_threshold', 0.85)
        self.declare_parameter('required_frames', 3)
        self.declare_parameter('max_frame_gap_s', 1.0)
        self.declare_parameter('status_timeout_s', 1.0)
        self.declare_parameter('publish_period_s', 0.2)

        self.status_topic = str(
            self.get_parameter('status_topic').value
        )
        self.target_info_topic = str(
            self.get_parameter('target_info_topic').value
        )
        self.target_found_topic = str(
            self.get_parameter('target_found_topic').value
        )
        self.similarity_threshold = float(
            self.get_parameter('similarity_threshold').value
        )
        self.required_frames = int(
            self.get_parameter('required_frames').value
        )
        self.max_frame_gap_s = float(
            self.get_parameter('max_frame_gap_s').value
        )
        self.status_timeout_s = float(
            self.get_parameter('status_timeout_s').value
        )
        self.publish_period_s = float(
            self.get_parameter('publish_period_s').value
        )

        self._validate_parameters()

        self.detector_status = 'UNKNOWN'
        self.last_status_time = None
        self.last_target_info_time = None
        self.confirm_count = 0
        self.target_found = False

        # TargetInfo is published BEST_EFFORT by balance_object_detect.
        detector_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # A transient-local latched result lets a search manager that starts
        # later immediately learn that the target was already confirmed.
        found_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.status_subscription = self.create_subscription(
            String,
            self.status_topic,
            self._status_callback,
            10,
        )
        self.target_subscription = self.create_subscription(
            TargetInfo,
            self.target_info_topic,
            self._target_callback,
            detector_qos,
        )
        self.publisher = self.create_publisher(
            Bool,
            self.target_found_topic,
            found_qos,
        )
        self.reset_service = self.create_service(
            Trigger,
            '/target_checker/reset',
            self._reset_callback,
        )
        self.timer = self.create_timer(
            self.publish_period_s,
            self._timer_callback,
        )

        self._publish_found()

        self.get_logger().info(
            'Balance target checker ready: '
            f'status={self.status_topic}, '
            f'target_info={self.target_info_topic}, '
            f'similarity>={self.similarity_threshold:.2f}, '
            f'frames={self.required_frames}'
        )

    def _validate_parameters(self):
        if not 0.0 <= self.similarity_threshold <= 1.0:
            raise ValueError(
                'similarity_threshold must be in [0, 1]'
            )
        if self.required_frames < 1:
            raise ValueError('required_frames must be >= 1')
        if self.max_frame_gap_s <= 0.0:
            raise ValueError('max_frame_gap_s must be positive')
        if self.status_timeout_s <= 0.0:
            raise ValueError('status_timeout_s must be positive')
        if self.publish_period_s <= 0.0:
            raise ValueError('publish_period_s must be positive')

    def _status_callback(self, msg):
        self.detector_status = msg.data.strip().upper()
        self.last_status_time = time.monotonic()

        if (
            not self.target_found
            and self.detector_status != 'TRACKING'
        ):
            self.confirm_count = 0

    def _status_is_fresh_tracking(self, now):
        return (
            self.detector_status == 'TRACKING'
            and self.last_status_time is not None
            and now - self.last_status_time
            <= self.status_timeout_s
        )

    def _target_callback(self, msg):
        if self.target_found:
            return

        now = time.monotonic()

        if (
            self.last_target_info_time is not None
            and now - self.last_target_info_time
            > self.max_frame_gap_s
        ):
            self.confirm_count = 0

        self.last_target_info_time = now

        similarity = float(msg.similarity)
        valid = (
            self._status_is_fresh_tracking(now)
            and bool(msg.detected)
            and math.isfinite(similarity)
            and similarity >= self.similarity_threshold
            and int(msg.width) > 0
            and int(msg.height) > 0
        )

        if not valid:
            self.confirm_count = 0
            return

        self.confirm_count += 1
        self.get_logger().info(
            'Valid target observation '
            f'({self.confirm_count}/{self.required_frames}): '
            f'similarity={similarity:.3f}, '
            f'center=({msg.center_x}, {msg.center_y}), '
            f'size=({msg.width}, {msg.height})'
        )

        if self.confirm_count >= self.required_frames:
            self.target_found = True
            self.get_logger().warn(
                'TARGET CONFIRMED: /target_found latched true'
            )
            self._publish_found()

    def _timer_callback(self):
        now = time.monotonic()

        # A partial confirmation sequence must not survive stale data.
        if (
            not self.target_found
            and self.confirm_count > 0
            and (
                not self._status_is_fresh_tracking(now)
                or self.last_target_info_time is None
                or now - self.last_target_info_time
                > self.max_frame_gap_s
            )
        ):
            self.confirm_count = 0

        self._publish_found()

    def _publish_found(self):
        msg = Bool()
        msg.data = self.target_found
        self.publisher.publish(msg)

    def _reset_callback(self, request, response):
        del request
        self.target_found = False
        self.confirm_count = 0
        self.last_target_info_time = None
        response.success = True
        response.message = (
            'Target confirmation latch and consecutive-frame '
            'counter were reset'
        )
        self._publish_found()
        self.get_logger().warn(
            'Target checker reset; detection confirmation enabled'
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TargetChecker()

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
