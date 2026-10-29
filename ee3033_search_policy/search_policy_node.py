"""Non-actuating ROS 2 readiness node for the learned search policy."""

from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from ee3033_search_policy.model import TinySearchPolicy


class SearchPolicyDryRunNode(Node):
    """Validate the model, map, TF and Nav2 without commanding motion."""

    def __init__(self):
        super().__init__("search_policy_dry_run")

        self.declare_parameter("weights_path", "")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("navigate_action", "/navigate_to_pose")
        self.declare_parameter("status_period_s", 2.0)

        self.map_topic = self.get_parameter("map_topic").value
        self.map_frame = self.get_parameter("map_frame").value
        self.robot_frame = self.get_parameter("robot_frame").value
        navigate_action = self.get_parameter("navigate_action").value
        status_period = float(self.get_parameter("status_period_s").value)

        configured_weights = self.get_parameter("weights_path").value
        if configured_weights:
            weights_path = Path(configured_weights).expanduser()
        else:
            share = Path(
                get_package_share_directory("ee3033_search_policy")
            )
            weights_path = (
                share / "models" / "tiny_search_policy_weights.npz"
            )

        self.policy = TinySearchPolicy(weights_path)
        probe_scores = self.policy.score(
            np.zeros((1, 12), dtype=np.float32)
        )
        self.get_logger().info(
            "Model validated: 12->32->16->1, 961 parameters, "
            f"SHA256={self.policy.sha256}, "
            f"probe={probe_scores[0]:.6f}"
        )

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self._map_callback,
            map_qos,
        )

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.navigate_client = ActionClient(
            self,
            NavigateToPose,
            navigate_action,
        )

        self.map_message = None
        self.last_status = None
        self.ready_announced = False
        self.timer = self.create_timer(
            status_period,
            self._check_readiness,
        )

        self.get_logger().warn(
            "DRY-RUN MODE: this node never sends navigation goals or cmd_vel."
        )
        self.get_logger().info(
            f"Waiting for map={self.map_topic}, "
            f"TF={self.map_frame}->{self.robot_frame}, "
            f"action={navigate_action}"
        )

    def _map_callback(self, message):
        first_map = self.map_message is None
        self.map_message = message
        if first_map:
            width_m = message.info.width * message.info.resolution
            height_m = message.info.height * message.info.resolution
            self.get_logger().info(
                "Map received: "
                f"{message.info.width}x{message.info.height} cells, "
                f"resolution={message.info.resolution:.3f} m/cell, "
                f"size={width_m:.2f}x{height_m:.2f} m"
            )

    def _check_readiness(self):
        map_ready = self.map_message is not None
        nav_ready = self.navigate_client.server_is_ready()
        tf_ready = False
        position_text = "unavailable"

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                Time(),
                timeout=Duration(seconds=0.2),
            )
            translation = transform.transform.translation
            tf_ready = True
            position_text = (
                f"({translation.x:.2f}, {translation.y:.2f}) m"
            )
        except TransformException:
            pass

        status = (map_ready, tf_ready, nav_ready)
        if status != self.last_status:
            self.get_logger().info(
                "Readiness: "
                f"map={'OK' if map_ready else 'WAIT'}, "
                f"TF={'OK' if tf_ready else 'WAIT'}, "
                f"Nav2={'OK' if nav_ready else 'WAIT'}, "
                f"position={position_text}"
            )
            self.last_status = status

        if all(status) and not self.ready_announced:
            self.get_logger().info(
                "ROS 2 dry-run readiness PASSED. "
                "No motion command was sent."
            )
            self.ready_announced = True


def main(args=None):
    rclpy.init(args=args)
    node = SearchPolicyDryRunNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
