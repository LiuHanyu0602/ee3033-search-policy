import json
import math
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool


class TargetChecker(Node):

    def __init__(self):
        super().__init__('target_checker')

        # ---------- Parameters ----------
        self.declare_parameter('target_class', 'bottle')
        self.declare_parameter('confidence_threshold', 0.70)
        self.declare_parameter('required_frames', 3)
        self.declare_parameter('max_frame_gap', 1.0)

        self.target_class = self.get_parameter(
            'target_class'
        ).value

        self.conf_threshold = float(
            self.get_parameter('confidence_threshold').value
        )

        self.required_frames = int(
            self.get_parameter('required_frames').value
        )

        self.max_frame_gap = float(
            self.get_parameter('max_frame_gap').value
        )

        if self.required_frames < 1:
            raise ValueError('required_frames must be >= 1')

        if not 0.0 <= self.conf_threshold <= 1.0:
            raise ValueError('confidence_threshold must be in [0, 1]')

        if self.max_frame_gap <= 0:
            raise ValueError('max_frame_gap must be positive')

        # ---------- States ----------
        self.target_found = False
        self.confirm_count = 0
        self.last_frame_time = None
        self.last_image_stamp = None

        # ---------- Subscribe to real YOLO NCNN ----------
        self.subscription = self.create_subscription(
            String,
            '/yolo/detections',
            self.detection_callback,
            10
        )

        # ---------- Publish target found ----------
        self.publisher = self.create_publisher(
            Bool,
            '/target_found',
            10
        )

        # Publish state at 5 Hz
        self.timer = self.create_timer(
            0.2,
            self.publish_status
        )

        self.get_logger().info(
            f'Target Checker started: '
            f'target={self.target_class}, '
            f'confidence>={self.conf_threshold}, '
            f'required_frames={self.required_frames}'
        )

    def detection_callback(self, msg):

        if self.target_found:
            return

        try:
            data = json.loads(msg.data)

            detections = data['detections']
            stamp = data['timestamp']

            image_stamp = (
                int(stamp['sec']),
                int(stamp['nanosec'])
            )

            if not isinstance(detections, list):
                raise ValueError('detections must be a list')

            if not 0 <= image_stamp[1] < 1_000_000_000:
                raise ValueError('invalid image timestamp')

        except (ValueError, TypeError, KeyError) as exc:
            self.get_logger().warn(
                f'Invalid YOLO message: {exc}'
            )
            self.confirm_count = 0
            return

        # Avoid counting the same camera frame repeatedly
        if image_stamp == self.last_image_stamp:
            return

        self.last_image_stamp = image_stamp

        now = time.monotonic()

        if (
            self.last_frame_time is not None
            and now - self.last_frame_time > self.max_frame_gap
        ):
            self.confirm_count = 0

        self.last_frame_time = now

        # Check whether this frame contains the target
        valid_target = False

        for det in detections:

            if not isinstance(det, dict):
                continue

            class_name = det.get('class_name', '')
            try:
                confidence = float(det.get('confidence', 0.0))
            except (ValueError, TypeError):
                continue

            if (
                class_name == self.target_class
                and math.isfinite(confidence)
                and confidence >= self.conf_threshold
            ):
                valid_target = True
                break

        # Consecutive frame confirmation
        if valid_target:
            self.confirm_count += 1

            self.get_logger().info(
                f'{self.target_class} detected '
                f'({self.confirm_count}/{self.required_frames})'
            )

        else:
            self.confirm_count = 0

        # Confirm target found
        if self.confirm_count >= self.required_frames:

            self.target_found = True

            self.get_logger().info(
                f'TARGET FOUND: {self.target_class}'
            )

            self.publish_status()

    def publish_status(self):

        result = Bool()
        result.data = self.target_found

        self.publisher.publish(result)


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

