"""Asynchronous Nav2 actions; planning never implicitly starts navigation."""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot, isfinite
from threading import RLock
from time import monotonic
from typing import Callable, Generic, Optional, TypeVar

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.task import Future


@dataclass(frozen=True)
class ActionResult:
    """Local outcome; cancellation acknowledgement is not a terminal outcome."""

    success: bool
    status: int = GoalStatus.STATUS_UNKNOWN
    error: str = ""
    latency_s: float = 0.0
    accepted: Optional[bool] = None
    timed_out: bool = False


@dataclass(frozen=True)
class PathResult(ActionResult):
    """Planning outcome; planning_time_s excludes transport and action overhead."""

    path: Optional[Path] = None
    path_length_m: float = 0.0
    planning_time_s: Optional[float] = None


@dataclass(frozen=True)
class NavigationResult(ActionResult):
    """Navigation outcome, deliberately separate from a computed path."""


ResultT = TypeVar("ResultT", bound=ActionResult)


def path_length(path: Path) -> float:
    """Return planar polyline length, rejecting empty or non-finite paths."""
    if not path.poses:
        raise ValueError("Nav2 returned an empty path")
    positions = [p.pose.position for p in path.poses]
    if not all(
        all(isfinite(v) for v in (p.x, p.y, p.z)) for p in positions
    ):
        raise ValueError("Nav2 path contains non-finite coordinates")
    length = sum(
        hypot(p1.x - p0.x, p1.y - p0.y)
        for p0, p1 in zip(positions, positions[1:])
    )
    if not isfinite(length):
        raise ValueError("Nav2 path length is not finite")
    return length


def _validate_pose(pose: PoseStamped) -> None:
    if not pose.header.frame_id:
        raise ValueError("Pose must have a nonempty frame_id")
    p, q = pose.pose.position, pose.pose.orientation
    if not all(isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
        raise ValueError("Pose contains non-finite values")
    norm = hypot(q.x, q.y, q.z, q.w)
    if abs(norm - 1.0) > 1e-3:
        raise ValueError("Pose quaternion must be normalized")


class ActionRequest(Generic[ResultT]):
    """Request handle: observe future; cancel asynchronously, never Future.cancel."""

    def __init__(self, channel: _ActionChannel[ResultT]):
        self.future: Future[ResultT] = Future()
        self._channel = channel
        self._started = monotonic()
        self._deadline = 0.0
        self._goal_handle: Optional[ClientGoalHandle] = None
        self._cancel_future: Optional[Future[bool]] = None
        self._cancel_sent = False
        self._accepted: Optional[bool] = None
        self._frame = ""
        self._sent = False
        self._remote_done = False

    def cancel(self) -> Future[bool]:
        """Request cancellation; True means acknowledged, not stopped/completed."""
        return self._channel.cancel(self)


class _ActionChannel(Generic[ResultT]):
    """Share lifecycle handling, while keeping each action's active slot separate."""

    def __init__(
        self, node: Node, action_type, action_name: str, result_type: type[ResultT],
    ):
        self.node = node
        self.result_type = result_type
        self._lock = RLock()
        self._group = MutuallyExclusiveCallbackGroup()
        self.client = ActionClient(
            node, action_type, action_name, callback_group=self._group,
        )
        self._active: Optional[ActionRequest[ResultT]] = None
        self._closed = False
        self._timer = node.create_timer(
            0.05, self._check_timeout, callback_group=self._group,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )
        self._timer.cancel()

    def submit(
        self, goal, timeout_s: float, frame: str,
        feedback_callback: Optional[Callable] = None,
    ) -> ActionRequest[ResultT]:
        if not isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("timeout_s must be finite and positive")
        with self._lock:
            request = ActionRequest(self)
            request._frame = frame
            request._deadline = request._started + timeout_s
            if self._closed:
                self._finish(request, error="Nav2 client has been closed")
            elif self._active is not None:
                self._finish(request, error="Action already has an active request")
            elif not self.client.server_is_ready():
                self._finish(request, error="Action server is not available")
            else:
                self._active = request
                self._timer.reset()

                def on_feedback(message):
                    if not request.future.done() and feedback_callback is not None:
                        feedback_callback(message)

                try:
                    request._sent = True
                    pending = self.client.send_goal_async(
                        goal,
                        feedback_callback=(
                            on_feedback if feedback_callback is not None else None
                        ),
                    )
                    pending.add_done_callback(
                        lambda future: self._goal_response(request, future),
                    )
                except (RuntimeError, ValueError) as exc:
                    request._remote_done = True
                    self._finish(request, error=f"Goal submission failed: {exc}")
            return request

    def _goal_response(self, request, future):
        with self._lock:
            self._expire(request)
            try:
                handle = future.result()
                request._accepted = handle.accepted
                if not handle.accepted:
                    request._remote_done = True
                    self._finish(request, error="Action goal was rejected")
                    self._cancel_ack(request, False)
                    return
                request._goal_handle = handle
                pending = handle.get_result_async()
                pending.add_done_callback(
                    lambda result: self._result_response(request, result),
                )
                # A timed-out request can be accepted after its local completion.
                if request.future.done() or request._cancel_future is not None:
                    self.cancel(request)
            except (RuntimeError, ValueError) as exc:
                if request._goal_handle is not None:
                    self.cancel(request)
                self._finish(request, error=f"Goal response failed: {exc}")
                self._cancel_ack(request, False)

    def _result_response(self, request, future):
        with self._lock:
            request._remote_done = True
            self._expire(request)
            try:
                response = future.result()
                self._finish(
                    request, status=response.status, payload=response.result,
                )
            except (RuntimeError, ValueError) as exc:
                self._finish(request, error=f"Action result failed: {exc}")
            self._cancel_ack(request, False)

    def _finish(
        self, request, status=GoalStatus.STATUS_UNKNOWN, payload=None,
        error="", timed_out=False,
    ):
        if request.future.done():
            return
        if self._active is request:
            self._active = None
            self._timer.cancel()
        fields = dict(
            success=not error and status == GoalStatus.STATUS_SUCCEEDED,
            status=status, error=error,
            latency_s=monotonic() - request._started,
            accepted=request._accepted, timed_out=timed_out,
        )
        if not error and status != GoalStatus.STATUS_SUCCEEDED:
            fields["error"] = f"Action finished with status {status}"
        if self.result_type is PathResult and payload is not None:
            try:
                path = payload.path
                length = path_length(path)
                if path.header.frame_id != request._frame:
                    raise ValueError("Returned path frame differs from request")
                if any(
                    p.header.frame_id not in ("", request._frame)
                    for p in path.poses
                ):
                    raise ValueError("Returned path contains inconsistent pose frames")
                duration = payload.planning_time
                if duration.sec < 0 or not 0 <= duration.nanosec < 1_000_000_000:
                    raise ValueError("Invalid Nav2 planning_time duration")
                fields.update(
                    path=path, path_length_m=length,
                    planning_time_s=duration.sec + duration.nanosec * 1e-9,
                )
            except ValueError as exc:
                fields.update(
                    success=False,
                    error="; ".join(filter(None, (fields["error"], str(exc)))),
                )
        if fields["error"]:
            self.node.get_logger().warning(fields["error"])
        request.future.set_result(self.result_type(**fields))

    def cancel(self, request: ActionRequest[ResultT]) -> Future[bool]:
        with self._lock:
            if request._cancel_future is None:
                request._cancel_future = Future()
            if request._remote_done or not request._sent:
                self._cancel_ack(request, False)
            elif request._goal_handle is None:
                if request._accepted is False:
                    self._cancel_ack(request, False)
            elif not request._cancel_sent:
                request._cancel_sent = True
                try:
                    pending = request._goal_handle.cancel_goal_async()
                    pending.add_done_callback(
                        lambda future: self._cancel_response(request, future),
                    )
                except (RuntimeError, ValueError) as exc:
                    self.node.get_logger().error(f"Cancellation failed: {exc}")
                    self._cancel_ack(request, False)
            return request._cancel_future

    def _cancel_response(self, request, future):
        with self._lock:
            try:
                acknowledged = bool(future.result().goals_canceling)
                if not acknowledged:
                    self.node.get_logger().warning(
                        "Server did not acknowledge goal cancellation",
                    )
                self._cancel_ack(request, acknowledged)
            except (RuntimeError, ValueError) as exc:
                self.node.get_logger().error(f"Cancellation response failed: {exc}")
                self._cancel_ack(request, False)

    @staticmethod
    def _cancel_ack(request, acknowledged):
        future = request._cancel_future
        if future is not None and not future.done():
            future.set_result(acknowledged)

    def _check_timeout(self):
        with self._lock:
            request = self._active
            if request is not None:
                self._expire(request)

    def _expire(self, request):
        if not request.future.done() and monotonic() >= request._deadline:
            self._finish(
                request, error="Action request timed out; cancellation is best-effort",
                timed_out=True,
            )
            self.cancel(request)

    def close(self):
        with self._lock:
            self._closed = True
            request = self._active
            if request is not None:
                self.cancel(request)
                self._finish(request, error="Nav2 client closed")
            self.node.destroy_timer(self._timer)
            self.client.destroy()


class Nav2Client:
    """Reusable planning API with a separately opted-in navigation API.

    Spin the owning node to progress requests. Each action allows one active
    local request; overlapping submissions return a failed result. Timeout
    releases the slot and requests best-effort cancellation, including late
    acceptance. It does not prove that the server stopped processing the goal.
    """

    def __init__(
        self, node: Node, planner_action: str = "/compute_path_to_pose",
        navigator_action: str = "/navigate_to_pose",
        enable_navigation: bool = False,
    ):
        self._node = node
        self._planner = _ActionChannel(
            node, ComputePathToPose, planner_action, PathResult,
        )
        self._navigator_action = navigator_action
        self._navigation_enabled = enable_navigation
        self._navigator: Optional[_ActionChannel[NavigationResult]] = None
        self._closed = False
        self._lock = RLock()

    def planner_is_ready(self) -> bool:
        """Inspect readiness without blocking the executor."""
        return not self._closed and self._planner.client.server_is_ready()

    def navigator_is_ready(self) -> bool:
        """Inspect readiness lazily; this does not enable navigation."""
        return not self._closed and self._navigation_channel().client.server_is_ready()

    def _navigation_channel(self):
        with self._lock:
            if self._closed:
                raise RuntimeError("Nav2 client has been closed")
            if self._navigator is None:
                self._navigator = _ActionChannel(
                    self._node, NavigateToPose, self._navigator_action,
                    NavigationResult,
                )
            return self._navigator

    def compute_path(
        self, start: PoseStamped, goal: PoseStamped,
        planner_id: str = "GridBased", timeout_s: float = 15.0,
    ) -> ActionRequest[PathResult]:
        """Plan from an explicit start, without moving the robot."""
        _validate_pose(start)
        _validate_pose(goal)
        if start.header.frame_id != goal.header.frame_id:
            raise ValueError("Start and goal must use the same frame")
        if not planner_id:
            raise ValueError("planner_id must not be empty")
        message = ComputePathToPose.Goal()
        message.start = start
        message.goal = goal
        message.use_start = True
        message.planner_id = planner_id
        return self._planner.submit(message, timeout_s, goal.header.frame_id)

    def navigate_to_pose(
        self, goal: PoseStamped, timeout_s: float = 120.0,
        feedback_callback: Optional[Callable] = None,
        behavior_tree: str = "",
    ) -> ActionRequest[NavigationResult]:
        """Execute only after explicit constructor opt-in and an explicit call.

        feedback_callback receives the ROS NavigateToPose feedback envelope.
        Do not submit a replacement after a timeout without checking robot state.
        """
        if not self._navigation_enabled:
            raise RuntimeError("Navigation is disabled; explicit opt-in is required")
        _validate_pose(goal)
        message = NavigateToPose.Goal()
        message.pose = goal
        message.behavior_tree = behavior_tree
        return self._navigation_channel().submit(
            message, timeout_s, goal.header.frame_id, feedback_callback,
        )

    def close(self):
        """Cancel active requests best-effort and release ROS entities."""
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._planner.close()
            if self._navigator is not None:
                self._navigator.close()