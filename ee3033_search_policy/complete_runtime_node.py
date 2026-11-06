"""Run the complete search runtime in one ROS context."""

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

from .continuous_observation_adapter import ContinuousObservationAdapter
from .euler_camera_tf_node import EulerCameraTf
from .guarded_search_manager import GuardedSearchManager
from .stamped_target_localizer_node import StampedTargetLocalizer
from .target_mission_coordinator import TargetMissionCoordinator


NODE_TYPES = (
    EulerCameraTf,
    ContinuousObservationAdapter,
    StampedTargetLocalizer,
    TargetMissionCoordinator,
    GuardedSearchManager,
)


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads=2)
    nodes = []
    try:
        for node_type in NODE_TYPES:
            node = node_type()
            nodes.append(node)
            executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        for node in reversed(nodes):
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
