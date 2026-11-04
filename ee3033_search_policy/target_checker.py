
"""Publish a latched target-found event from confirmed YOLO observations."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.qos import DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from ee3033_search_policy_interfaces.msg import TargetObservation


class TargetChecker(Node):
    def __init__(self):
        super().__init__('target_checker')

        self.declare_parameter(
            'observation_topic',
            '/detector/target_observation'
        )
        self.declare_parameter(
            'target_found_topic',
            '/target_found'
        )
        self.declare_parameter('target_class_id', 39)
        self.declare_parameter('confidence_threshold', 0.70)

        observation_topic = self.get_parameter(
            'observation_topic'
        ).value
        found_topic = self.get_parameter(
            'target_found_topic'
        ).value

        self.class_id = int(
            self.get_parameter('target_class_id').value
        )
        self.threshold = float(
            self.get_parameter('confidence_threshold').value
        )

        self.target_found = False
        self.active_episode_id = None

        found_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.publisher = self.create_publisher(
            Bool, found_topic, found_qos
        )

        self.subscription = self.create_subscription(
            TargetObservation,
            observation_topic,
            self.on_observation,
            10,
        )

        self.reset_service = self.create_service(
            Trigger,
            '/target_checker/reset',
            self.on_reset,
        )

        self.publish_found()

    def on_observation(self, msg):
        if self.target_found:
            return

        if not msg.confirmed:
            return

        if msg.class_id != self.class_id:
            return

        if msg.class_name.strip().casefold() != 'bottle':
            return

        if msg.confidence < self.threshold:
            return

        if not msg.episode_id:
            return

        if self.active_episode_id is None:
            self.active_episode_id = msg.episode_id
        elif msg.episode_id != self.active_episode_id:
            return

        self.target_found = True
        self.publish_found()
        self.get_logger().warn(
            'Bottle confirmed; target_found=True'
        )

    def publish_found(self):
        msg = Bool()
        msg.data = self.target_found
        self.publisher.publish(msg)

    def on_reset(self, request, response):
        self.target_found = False
        self.active_episode_id = None
        self.publish_found()
        response.success = True
        response.message = 'Target checker reset'
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
