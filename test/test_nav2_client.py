"""Isolated Action lifecycle tests with fakes, not ROS integration tests."""

from concurrent.futures import Future
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS

import pytest


def pose(x=0.0, y=0.0, frame="map"):
    return NS(
        header=NS(frame_id=frame),
        pose=NS(
            position=NS(x=x, y=y, z=0.0),
            orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )


def payload(points=((0.0, 0.0), (3.0, 4.0)), frame="map"):
    return NS(
        path=NS(header=NS(frame_id=frame), poses=[pose(*p) for p in points]),
        planning_time=NS(sec=0, nanosec=1_100_000),
    )


class FakeHandle:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.result_future = Future()
        self.cancel_future = Future()
        self.cancel_calls = 0

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancel_calls += 1
        return self.cancel_future


class FakeClient:
    instances = []

    def __init__(self, node, action_type, action_name, **kwargs):
        self.name = action_name
        self.ready = True
        self.sent = []
        self.destroyed = False
        self.instances.append(self)

    def server_is_ready(self):
        return self.ready

    def send_goal_async(self, goal, feedback_callback=None):
        future = Future()
        self.sent.append((goal, future, feedback_callback))
        return future

    def destroy(self):
        self.destroyed = True


class FakeNode:
    def __init__(self, *args, **kwargs):
        self.logs = []
        self.timers = []
        self.parameters = {}
        self.publishers = []

    def get_logger(self):
        return NS(
            info=self.logs.append, warning=self.logs.append, error=self.logs.append,
        )

    def create_timer(self, period, callback, **kwargs):
        timer = NS(callback=callback, canceled=False, reset_count=0)
        timer.cancel = lambda: setattr(timer, "canceled", True)
        def reset():
            timer.canceled = False
            timer.reset_count += 1
        timer.reset = reset
        self.timers.append(timer)
        return timer

    def destroy_timer(self, timer):
        self.timers.remove(timer)

    def declare_parameter(self, name, default):
        self.parameters[name] = default

    def get_parameter(self, name):
        return NS(value=self.parameters[name])

    def create_publisher(self, message_type, topic, qos):
        publisher = NS(topic=topic, qos=qos, messages=[])
        publisher.publish = publisher.messages.append
        self.publishers.append(publisher)
        return publisher

    def create_subscription(self, message_type, topic, callback, qos):
        return NS(topic=topic, callback=callback, qos=qos)

    def get_clock(self):
        return NS(now=lambda: NS(to_msg=lambda: NS(sec=0, nanosec=0)))

    def destroy_node(self):
        return True


@pytest.fixture
def api(monkeypatch):
    """Load a private copy so fake ROS modules never contaminate real imports."""
    status = NS(
        STATUS_UNKNOWN=0, STATUS_SUCCEEDED=4, STATUS_CANCELED=5, STATUS_ABORTED=6,
    )
    modules = {
        "action_msgs.msg": {"GoalStatus": status},
        "builtin_interfaces.msg": {"Duration": NS},
        "geometry_msgs.msg": {"PoseStamped": object},
        "nav2_msgs.action": {
            "ComputePathToPose": NS(Goal=NS), "NavigateToPose": NS(Goal=NS),
            "Spin": NS(Goal=NS),
        },
        "nav_msgs.msg": {"Path": object},
        "rclpy.action": {"ActionClient": FakeClient},
        "rclpy.action.client": {"ClientGoalHandle": FakeHandle},
        "rclpy.callback_groups": {"MutuallyExclusiveCallbackGroup": object},
        "rclpy.clock": {
            "Clock": lambda **kw: None, "ClockType": NS(STEADY_TIME=1),
        },
        "rclpy.node": {"Node": FakeNode},
        "rclpy.task": {"Future": Future},
    }
    for name, exports in modules.items():
        module = ModuleType(name)
        module.__dict__.update(exports)
        monkeypatch.setitem(sys.modules, name, module)
    location = Path(__file__).resolve().parents[1] / "ee3033_search_policy" / "nav2_client.py"
    spec = importlib.util.spec_from_file_location("_nav2_client_under_test", location)
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, module)
    spec.loader.exec_module(module)
    FakeClient.instances = []
    return module


def accepted(client):
    handle = FakeHandle()
    client.sent[-1][1].set_result(handle)
    return handle


def finish(handle, status=4, result=None):
    handle.result_future.set_result(
        NS(status=status, result=payload() if result is None else result),
    )


def test_planning_success_and_sequential_reuse(api):
    node = FakeNode()
    client = api.Nav2Client(node)
    assert node.timers[0].canceled
    request = client.compute_path(pose(), pose(3.0, 4.0))
    assert not node.timers[0].canceled
    assert node.timers[0].reset_count == 1
    action = FakeClient.instances[0]
    assert len(FakeClient.instances) == 1
    goal = action.sent[0][0]
    assert goal.use_start and goal.planner_id == "GridBased"
    handle = accepted(action)
    assert not request.future.done()
    finish(handle)
    result = request.future.result()
    assert result.success and result.status == 4 and result.accepted
    assert result.path_length_m == 5.0
    assert result.planning_time_s == pytest.approx(0.0011)
    assert result.latency_s >= 0
    assert node.timers[0].canceled
    assert not request.cancel().result()
    second = client.compute_path(pose(), pose(2.0, 1.0))
    assert not second.future.done() and len(action.sent) == 2
    assert not node.timers[0].canceled
    assert node.timers[0].reset_count == 2


def test_spin_client_sends_relative_yaw_and_requires_remote_success(api):
    node = FakeNode()
    client = api.SpinClient(node)
    assert client.is_ready()
    request = client.spin(-0.42, time_allowance_s=8.25, timeout_s=10.0)
    action = FakeClient.instances[0]
    goal = action.sent[-1][0]
    assert goal.target_yaw == pytest.approx(-0.42)
    assert (goal.time_allowance.sec, goal.time_allowance.nanosec) == (8, 250_000_000)
    handle = accepted(action)
    assert not request.future.done()
    finish(handle, status=4, result=NS())
    assert request.future.result().success


def test_spin_timeout_must_exceed_allowance(api):
    client = api.SpinClient(FakeNode())
    with pytest.raises(ValueError):
        client.spin(0.2, time_allowance_s=8.0, timeout_s=8.0)


def test_busy_does_not_send_or_displace_first(api):
    client = api.Nav2Client(FakeNode())
    first = client.compute_path(pose(), pose(1.0))
    second = client.compute_path(pose(), pose(2.0))
    assert "active" in second.future.result().error
    assert not second.cancel().result()
    assert len(FakeClient.instances[0].sent) == 1
    handle = accepted(FakeClient.instances[0])
    finish(handle)
    assert first.future.result().success


def test_unavailable_and_rejected(api):
    client = api.Nav2Client(FakeNode())
    action = FakeClient.instances[0]
    action.ready = False
    assert not client.planner_is_ready()
    request = client.compute_path(pose(), pose(1.0))
    assert "not available" in request.future.result().error
    assert not request.cancel().result()
    action.ready = True
    request = client.compute_path(pose(), pose(1.0))
    action.sent[-1][1].set_result(FakeHandle(accepted=False))
    assert request.future.result().accepted is False
    assert "rejected" in request.future.result().error


@pytest.mark.parametrize("status,points", [
    (6, ((0.0, 0.0), (1.0, 0.0))), (4, ()), (6, ()),
    (4, ((float("nan"), 0.0),)),
])
def test_failed_or_invalid_paths(api, status, points):
    client = api.Nav2Client(FakeNode())
    request = client.compute_path(pose(), pose(1.0))
    finish(accepted(FakeClient.instances[0]), status, payload(points))
    result = request.future.result()
    assert not result.success and result.status == status and result.error


def test_wrong_path_frame_and_bad_duration(api):
    client = api.Nav2Client(FakeNode())
    for data in (payload(frame="odom"), payload()):
        if data.path.header.frame_id == "map":
            data.planning_time.nanosec = -1
        request = client.compute_path(pose(), pose(1.0))
        finish(accepted(FakeClient.instances[0]), result=data)
        assert not request.future.result().success


def test_cancel_before_acceptance_waits_for_terminal_result(api):
    client = api.Nav2Client(FakeNode())
    request = client.compute_path(pose(), pose(1.0))
    cancellation = request.cancel()
    assert not cancellation.done()
    handle = accepted(FakeClient.instances[0])
    assert handle.cancel_calls == 1
    handle.cancel_future.set_result(NS(goals_canceling=[NS()]))
    assert cancellation.result()
    assert not request.future.done()
    finish(handle, status=5)
    assert request.future.result().status == 5
    assert not request.future.result().success
    assert request.cancel() is cancellation


def test_timeout_cancels_late_acceptance_without_corrupting_new_request(api, monkeypatch):
    now = [100.0]
    monkeypatch.setattr(api, "monotonic", lambda: now[0])
    node = FakeNode()
    client = api.Nav2Client(node)
    action = FakeClient.instances[0]
    old = client.compute_path(pose(), pose(1.0), timeout_s=1.0)
    old_ack = action.sent[0][1]
    now[0] = 101.01
    node.timers[0].callback()
    assert old.future.result().timed_out
    new = client.compute_path(pose(), pose(2.0))
    late = FakeHandle()
    old_ack.set_result(late)
    assert late.cancel_calls == 1
    finish(late, status=5)
    assert old.future.result().timed_out
    assert not new.future.done()
    finish(accepted(action))
    assert new.future.result().success


def test_accepted_timeout_and_cancel_refusal(api, monkeypatch):
    now = [0.0]
    monkeypatch.setattr(api, "monotonic", lambda: now[0])
    node = FakeNode()
    client = api.Nav2Client(node)
    request = client.compute_path(pose(), pose(1.0), timeout_s=1.0)
    handle = accepted(FakeClient.instances[0])
    now[0] = 2
    node.timers[0].callback()
    handle.cancel_future.set_result(NS(goals_canceling=[]))
    assert not request.cancel().result()
    assert request.future.result().timed_out


def test_late_result_cannot_win_over_deadline_if_timer_has_not_run(api, monkeypatch):
    now = [0.0]
    monkeypatch.setattr(api, "monotonic", lambda: now[0])
    client = api.Nav2Client(FakeNode())
    request = client.compute_path(pose(), pose(1.0), timeout_s=1.0)
    handle = accepted(FakeClient.instances[0])
    now[0] = 1.01
    finish(handle)
    assert request.future.result().timed_out
    assert not request.future.result().success


def test_late_acceptance_checks_deadline_without_timer(api, monkeypatch):
    now = [0.0]
    monkeypatch.setattr(api, "monotonic", lambda: now[0])
    client = api.Nav2Client(FakeNode())
    request = client.compute_path(pose(), pose(1.0), timeout_s=1.0)
    now[0] = 1.01
    handle = accepted(FakeClient.instances[0])
    assert request.future.result().timed_out
    assert handle.cancel_calls == 1


def test_transport_failure_is_reported(api):
    node = FakeNode()
    client = api.Nav2Client(node)
    request = client.compute_path(pose(), pose(1.0))
    FakeClient.instances[0].sent[0][1].set_exception(RuntimeError("transport failed"))
    assert "transport failed" in request.future.result().error
    assert node.logs


def test_result_transport_failure_releases_slot(api):
    client = api.Nav2Client(FakeNode())
    request = client.compute_path(pose(), pose(1.0))
    accepted(FakeClient.instances[0]).result_future.set_exception(
        RuntimeError("result transport failed"),
    )
    assert "result transport failed" in request.future.result().error
    assert not client.compute_path(pose(), pose(1.0)).future.done()


def test_close_is_idempotent_and_cancels_active_request(api):
    client = api.Nav2Client(FakeNode())
    request = client.compute_path(pose(), pose(1.0))
    handle = accepted(FakeClient.instances[0])
    client.close()
    client.close()
    assert handle.cancel_calls == 1
    assert "closed" in request.future.result().error
    assert "closed" in client.compute_path(pose(), pose(1.0)).future.result().error


def test_navigation_requires_opt_in_and_is_separate(api):
    node = FakeNode()
    client = api.Nav2Client(node)
    with pytest.raises(RuntimeError, match="disabled"):
        client.navigate_to_pose(pose())
    assert len(FakeClient.instances) == 1
    assert client.navigator_is_ready()
    with pytest.raises(RuntimeError, match="disabled"):
        client.navigate_to_pose(pose())
    assert not FakeClient.instances[1].sent
    client.close()
    assert not client.planner_is_ready()
    assert not client.navigator_is_ready()
    assert not node.timers


def test_explicit_navigation_feedback_result_and_cancel(api):
    client = api.Nav2Client(FakeNode(), enable_navigation=True)
    feedback = []
    request = client.navigate_to_pose(pose(1.0), feedback_callback=feedback.append)
    action = FakeClient.instances[1]
    action.sent[0][2](NS(distance_remaining=1.0))
    assert len(feedback) == 1
    handle = accepted(action)
    cancellation = request.cancel()
    handle.cancel_future.set_result(NS(goals_canceling=[NS()]))
    assert cancellation.result() and not request.future.done()
    finish(handle, status=5, result=NS())
    assert isinstance(request.future.result(), api.NavigationResult)
    assert not request.future.result().success
    action.sent[0][2](NS(distance_remaining=0.0))
    assert len(feedback) == 1


def test_navigation_success_and_planning_have_independent_slots(api):
    client = api.Nav2Client(FakeNode(), enable_navigation=True)
    planned = client.compute_path(pose(), pose(1.0))
    navigated = client.navigate_to_pose(pose(1.0))
    finish(accepted(FakeClient.instances[1]), result=NS())
    assert navigated.future.result().success
    assert not planned.future.done()
    finish(accepted(FakeClient.instances[0]))
    assert planned.future.result().success


def test_invalid_input_never_sends(api):
    client = api.Nav2Client(FakeNode())
    bad = pose()
    bad.pose.orientation.w = 0.0
    for start, goal, options in (
        (bad, pose(), {}),
        (pose(), pose(frame="odom"), {}),
        (pose(), pose(), {"timeout_s": 0}),
        (pose(), pose(), {"planner_id": ""}),
    ):
        with pytest.raises(ValueError):
            client.compute_path(start, goal, **options)
    assert not FakeClient.instances[0].sent


def test_extreme_path_coordinates_are_reported_not_overflowed(api):
    with pytest.raises(ValueError, match="not finite"):
        api.path_length(payload(((-1e308, 0.0), (1e308, 0.0))).path)


def test_path_pose_frame_is_validated(api):
    client = api.Nav2Client(FakeNode())
    data = payload()
    data.path.poses[0].header.frame_id = "odom"
    request = client.compute_path(pose(), pose(1.0))
    finish(accepted(FakeClient.instances[0]), result=data)
    assert "inconsistent" in request.future.result().error


class FakeMarker:
    ARROW = 0
    ADD = 0
    TEXT_VIEW_FACING = 9

    def __init__(self):
        self.pose = pose().pose
        self.scale = NS()
        self.color = NS()


@pytest.fixture
def planner_api(api, monkeypatch):
    """Run the real planner-node controller against in-memory ROS entities."""
    modules = {
        "rclpy": {},
        "geometry_msgs.msg": {"PoseStamped": pose},
        "nav_msgs.msg": {"OccupancyGrid": object, "Path": object},
        "rclpy.qos": {
            "QoSProfile": NS,
            "DurabilityPolicy": NS(TRANSIENT_LOCAL="transient_local"),
            "ReliabilityPolicy": NS(RELIABLE="reliable"),
        },
        "visualization_msgs.msg": {
            "Marker": FakeMarker, "MarkerArray": lambda: NS(markers=[]),
        },
    }
    for name, exports in modules.items():
        module = ModuleType(name)
        module.__dict__.update(exports)
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.setitem(sys.modules, "ee3033_search_policy.nav2_client", api)
    location = (
        Path(__file__).resolve().parents[1]
        / "ee3033_search_policy" / "planner_test_node.py"
    )
    spec = importlib.util.spec_from_file_location("_planner_node_under_test", location)
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, module)
    spec.loader.exec_module(module)
    return module


def planner_map():
    return NS(
        header=NS(frame_id="map"),
        info=NS(
            width=20, height=20, resolution=0.2,
            origin=NS(
                position=NS(x=-2.0, y=-2.0, z=0.0),
                orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        ),
        data=[0] * 400,
    )


def test_node_one_shot_retained_visualization_and_no_navigation(planner_api):
    node = planner_api.PlannerTestNode()
    assert len(FakeClient.instances) == 1
    assert FakeClient.instances[0].name == "/compute_path_to_pose"
    assert {p.topic for p in node.publishers} == {
        "/search/debug/start", "/search/debug/goal",
        "/search/debug/path", "/search/debug/markers",
    }
    assert all(p.qos.durability == "transient_local" for p in node.publishers)
    assert all(p.qos.reliability == "reliable" for p in node.publishers)
    assert node._subscription.qos.durability == "transient_local"
    node._on_map(planner_map())
    node._tick()
    action = FakeClient.instances[0]
    assert len(action.sent) == 1
    assert action.sent[0][0].use_start
    assert node._timer.canceled
    assert len(node._start_pub.messages) == len(node._goal_pub.messages) == 1
    markers = node._markers_pub.messages[0].markers
    assert len(markers) == 4
    assert {m.text for m in markers if m.type == FakeMarker.TEXT_VIEW_FACING} == {
        "START", "GOAL",
    }
    finish(accepted(action))
    assert len(node._path_pub.messages) == 1
    assert any("Planning SUCCEEDED" in line for line in node.logs)
    node._on_map(planner_map())
    node._tick()
    assert len(action.sent) == 1
    node.destroy_node()
    assert action.destroyed


@pytest.mark.parametrize("failure", ["missing_map", "missing_planner", "bad_map"])
def test_node_readiness_and_map_failures_are_non_actuating(planner_api, failure):
    node = planner_api.PlannerTestNode()
    if failure != "missing_map":
        message = planner_map()
        if failure == "bad_map":
            message.data = [0]
        node._on_map(message)
    if failure == "missing_planner":
        FakeClient.instances[0].ready = False
    node._started -= 31
    node._tick()
    assert node._finished and node._timer.canceled
    assert not FakeClient.instances[0].sent
    assert not node._path_pub.messages


def test_node_planning_failure_does_not_publish_path(planner_api):
    node = planner_api.PlannerTestNode()
    node._on_map(planner_map())
    node._tick()
    finish(accepted(FakeClient.instances[0]), status=6, result=payload(()))
    assert node._finished and not node._path_pub.messages
    assert any("Planning FAILED: status=6" in line for line in node.logs)


def test_invalid_startup_configuration_returns_failure(planner_api, monkeypatch):
    logs, shutdowns = [], []

    def invalid_node():
        raise ValueError("bad clearance")

    monkeypatch.setattr(planner_api, "PlannerTestNode", invalid_node)
    for name, value in {
        "init": lambda **kwargs: None,
        "ok": lambda: True,
        "shutdown": lambda: shutdowns.append(True),
        "logging": NS(get_logger=lambda name: NS(error=logs.append)),
    }.items():
        monkeypatch.setattr(planner_api.rclpy, name, value, raising=False)
    assert planner_api.main() == 1
    assert shutdowns == [True]
    assert "bad clearance" in logs[0]
