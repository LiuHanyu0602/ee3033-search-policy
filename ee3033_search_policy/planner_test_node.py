"""One-shot, non-actuating Nav2 planning test with retained RViz diagnostics."""

from math import atan2, cos, isfinite, sin
from time import monotonic

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from ee3033_search_policy.map_utils import (
    GridGeometry,
    occupancy_array,
    safe_candidate_mask,
    select_test_positions,
)
from ee3033_search_policy.nav2_client import Nav2Client


class PlannerTestNode(Node):
    """Select connected map points, request one path, and never command motion."""

    def __init__(self):
        super().__init__("planner_test")
        defaults = {
            "map_topic": "/map",
            "planner_action": "/compute_path_to_pose",
            "planner_id": "GridBased",
            "clearance_m": 0.20,
            "min_separation_m": 1.0,
            "free_threshold": 20,
            "occupied_threshold": 65,
            "readiness_timeout_s": 30.0,
            "request_timeout_s": 15.0,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        self.settings = {
            name: self.get_parameter(name).value for name in defaults
        }
        for name in ("map_topic", "planner_action", "planner_id"):
            if not self.settings[name]:
                raise ValueError(f"{name} must not be empty")
        for name in (
            "min_separation_m", "readiness_timeout_s", "request_timeout_s",
        ):
            value = self.settings[name]
            if not isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        clearance = self.settings["clearance_m"]
        if not isfinite(clearance) or clearance < 0:
            raise ValueError("clearance_m must be finite and nonnegative")
        if not (
            0 <= self.settings["free_threshold"]
            < self.settings["occupied_threshold"] <= 100
        ):
            raise ValueError("Require 0 <= free_threshold < occupied_threshold <= 100")

        retained = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._start_pub = self.create_publisher(
            PoseStamped, "/search/debug/start", retained,
        )
        self._goal_pub = self.create_publisher(
            PoseStamped, "/search/debug/goal", retained,
        )
        self._path_pub = self.create_publisher(
            Path, "/search/debug/path", retained,
        )
        self._markers_pub = self.create_publisher(
            MarkerArray, "/search/debug/markers", retained,
        )
        self._map = None
        self._finished = False
        self._submitted = False
        self._started = monotonic()
        self._subscription = self.create_subscription(
            OccupancyGrid, self.settings["map_topic"], self._on_map, retained,
        )
        # No NavigateToPose client is created by this planning-only node.
        self._nav2 = Nav2Client(
            self, planner_action=self.settings["planner_action"],
        )
        self._timer = self.create_timer(
            0.2, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME),
        )
        self.get_logger().warning(
            "PLANNING ONLY: no navigation goals, velocity commands, or waypoint loop.",
        )
        self.get_logger().info(
            f"Waiting for map={self.settings['map_topic']} and "
            f"planner={self.settings['planner_action']}. "
            "Clearance is a test-point filter, not a verified robot footprint.",
        )

    def _on_map(self, message):
        if not self._submitted and not self._finished:
            self._map = message

    def _fail(self, message):
        self._finished = True
        self._timer.cancel()
        self.get_logger().error(message)

    def _tick(self):
        if self._finished or self._submitted:
            return
        map_ready = self._map is not None
        planner_ready = self._nav2.planner_is_ready()
        if not map_ready or not planner_ready:
            if monotonic() - self._started >= self.settings["readiness_timeout_s"]:
                self._fail(
                    "Readiness timed out: "
                    f"map={'OK' if map_ready else 'MISSING'}, "
                    f"planner={'OK' if planner_ready else 'MISSING'}. "
                    "Check map durability, lifecycle state, action name and ROS network.",
                )
            return
        try:
            message = self._map
            if not message.header.frame_id:
                raise ValueError("Map frame_id must not be empty")
            geometry = GridGeometry.from_message(message)
            values = occupancy_array(message)
            safe = safe_candidate_mask(
                values, geometry.resolution,
                clearance_m=self.settings["clearance_m"],
                free_threshold=self.settings["free_threshold"],
                occupied_threshold=self.settings["occupied_threshold"],
            )
            start_xy, goal_xy = select_test_positions(
                safe, geometry, self.settings["min_separation_m"],
            )
            yaw = atan2(goal_xy[1] - start_xy[1], goal_xy[0] - start_xy[0])
            start = self._pose(start_xy, yaw, message.header.frame_id)
            goal = self._pose(goal_xy, yaw, message.header.frame_id)
            self._start_pub.publish(start)
            self._goal_pub.publish(goal)
            self._publish_markers(start, goal)
            self.get_logger().info(
                f"Map: {geometry.width}x{geometry.height}, "
                f"resolution={geometry.resolution:.3f} m/cell; "
                f"start=({start_xy[0]:.3f}, {start_xy[1]:.3f}), "
                f"goal=({goal_xy[0]:.3f}, {goal_xy[1]:.3f}), "
                f"frame={message.header.frame_id}",
            )
            self._submitted = True
            self._timer.cancel()
            request = self._nav2.compute_path(
                start, goal, planner_id=self.settings["planner_id"],
                timeout_s=self.settings["request_timeout_s"],
            )
            request.future.add_done_callback(self._on_result)
        except (ValueError, TypeError) as exc:
            self._fail(f"Planning test could not start: {exc}")

    def _pose(self, xy, yaw, frame):
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = xy
        pose.pose.orientation.z = sin(yaw / 2)
        pose.pose.orientation.w = cos(yaw / 2)
        return pose

    def _publish_markers(self, start, goal):
        markers = MarkerArray()
        for index, (pose, label, rgb) in enumerate((
            (start, "START", (0.1, 0.9, 0.2)),
            (goal, "GOAL", (1.0, 0.3, 0.1)),
        )):
            arrow = Marker()
            arrow.header = pose.header
            arrow.ns = "planner_test"
            arrow.id = index
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose = pose.pose
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.35, 0.08, 0.08
            arrow.color.r, arrow.color.g, arrow.color.b = rgb
            arrow.color.a = 1.0
            markers.markers.append(arrow)
            text = Marker()
            text.header = pose.header
            text.ns = "planner_test_labels"
            text.id = index
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = pose.pose.position.x
            text.pose.position.y = pose.pose.position.y
            text.pose.position.z = 0.4
            text.pose.orientation.w = 1.0
            text.scale.z = 0.2
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            text.text = label
            markers.markers.append(text)
        self._markers_pub.publish(markers)

    def _on_result(self, future):
        result = future.result()
        self._finished = True
        if not result.success:
            self._fail(
                f"Planning FAILED: status={result.status}, "
                f"accepted={result.accepted}, timed_out={result.timed_out}, "
                f"request_latency={result.latency_s * 1000:.2f} ms; {result.error}",
            )
            return
        self._path_pub.publish(result.path)
        self.get_logger().info(
            f"Planning SUCCEEDED: status={result.status}, "
            f"poses={len(result.path.poses)}, length={result.path_length_m:.3f} m, "
            f"planning_time={result.planning_time_s * 1000:.2f} ms, "
            f"request_latency={result.latency_s * 1000:.2f} ms. "
            "Debug results retained while this node is running. No motion requested.",
        )

    def destroy_node(self):
        self._nav2.close()
        return super().destroy_node()


def main(args=None):
    """Run until interrupted so transient-local publishers remain available."""
    rclpy.init(args=args)
    node = None
    try:
        node = PlannerTestNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except (ValueError, TypeError) as exc:
        rclpy.logging.get_logger("planner_test").error(f"Invalid configuration: {exc}")
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
