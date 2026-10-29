"""Offline search-readiness regressions with isolated ROS test doubles."""

from concurrent.futures import Future
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS
from unittest.mock import Mock

import numpy as np
import pytest

from ee3033_search_policy.map_utils import GridGeometry, grid_to_world


@pytest.fixture
def api(monkeypatch):
    modules = {
        "rclpy": {},
        "ament_index_python.packages": {"get_package_share_directory": Mock()},
        "geometry_msgs.msg": {"PoseStamped": object},
        "nav_msgs.msg": {"OccupancyGrid": object},
        "rclpy.duration": {"Duration": lambda **kwargs: None},
        "rclpy.node": {"Node": object},
        "rclpy.qos": {
            "DurabilityPolicy": NS(TRANSIENT_LOCAL=1),
            "ReliabilityPolicy": NS(RELIABLE=1), "QoSProfile": object,
        },
        "rclpy.time": {"Time": lambda: None},
        "std_msgs.msg": {"Bool": object, "Float32": object, "String": object},
        "std_srvs.srv": {"Trigger": object},
        "tf2_ros": {
            "Buffer": object, "TransformException": type("TransformException", (Exception,), {}),
            "TransformListener": object,
        },
        "ee3033_search_policy.nav2_client": {"Nav2Client": object},
        "ee3033_search_policy.target_stop_guard": {"TargetStopGuard": object},
    }
    for name, exports in modules.items():
        module = ModuleType(name)
        module.__dict__.update(exports)
        monkeypatch.setitem(sys.modules, name, module)
    path = Path(__file__).resolve().parents[1] / "ee3033_search_policy" / "search_manager_node.py"
    spec = importlib.util.spec_from_file_location(
        "ee3033_search_policy._search_manager_under_test", path,
    )
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, module)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "time", NS(monotonic=lambda: 100.0))
    return module


@pytest.fixture
def manager(api):
    node = api.SearchManager.__new__(api.SearchManager)
    node.geometry = GridGeometry(10, 10, 1.0)
    node.map_frame, node.robot_frame = "map", "base_footprint"
    node.tf_buffer = Mock()
    node.tf_buffer.lookup_transform.return_value = NS(transform=NS(
        translation=NS(x=1.5, y=2.5),
        rotation=NS(x=0.0, y=0.0, z=0.0, w=1.0),
    ))
    node.get_logger = Mock(return_value=Mock())
    node._status, node._return_status, node._progress = Mock(), Mock(), Mock()
    node._begin_decision, node._plan_next, node._failure = Mock(), Mock(), Mock()
    node._try_return_home, node._finish_observation = Mock(), Mock()
    node._cancel = Mock()
    node.return_pending = node.finished = node.busy = False
    node.return_request = None
    node.single_goal_mode = True
    node.max_navigation_goals = 1
    node.navigation_goal_count = 0
    node.operator_cancel_requested = False
    node.selected_index = None
    node.observation_deadline = None
    node.failure_counts = np.zeros(2, dtype=np.int32)
    node.failure_times = np.full(2, -np.inf)
    node.observation_deadline = None
    node.retry_after = 0.0
    node.mission_state = "SEARCHING"
    node.coverage_target = 0.995
    node.memory = NS(coverage_fraction=0.1, resume=Mock(), pause=Mock(), reset=Mock(),
                     mark_observation_complete=Mock())
    node.home_pose = (1.5, 2.5, 0.0)
    node.stop_guard = NS(may_dispatch=Mock(return_value=True), stopped=False,
                         active_request=None, reset=Mock())
    node.nav2 = NS(planner_is_ready=Mock(return_value=True),
                   navigate_to_pose=Mock())
    node.active_plan = None
    node.path_distances = np.full(2, np.inf)
    node.candidates = NS(visibility_masks=np.zeros((2, 10, 10), dtype=bool))
    node.plan_cursor = 0
    return node


@pytest.mark.parametrize("x,y", [
    (-0.1, 1.0), (10.0, 1.0), (1.0, -0.1), (1.0, 10.0),
    (float("nan"), 1.0), (1.0, float("inf")),
])
def test_invalid_robot_position_cannot_plan_or_record_home(manager, x, y):
    manager.home_pose = None
    manager.tf_buffer.lookup_transform.return_value.transform.translation = NS(x=x, y=y)
    manager._tick()
    manager._status.assert_called_once_with("ROBOT_POSE_OUTSIDE_MAP")
    manager.get_logger().error.assert_called_once()
    manager._begin_decision.assert_not_called()
    manager.nav2.planner_is_ready.assert_not_called()
    manager.nav2.navigate_to_pose.assert_not_called()
    assert manager.home_pose is None


def test_mission_start_does_not_record_an_invalid_home(manager):
    manager.mission_state, manager.home_pose = "IDLE", None
    manager.tf_buffer.lookup_transform.return_value.transform.translation.x = -1.0
    manager._mission_cb(NS(data="SEARCHING"))
    assert manager.home_pose is None
    manager._status.assert_called_once_with("ROBOT_POSE_OUTSIDE_MAP")
    manager._return_status.assert_not_called()


@pytest.mark.parametrize("missing", ["map", "tf", "planner"])
def test_missing_readiness_blocks_planning(api, manager, missing):
    if missing == "map":
        manager.geometry = None
    elif missing == "tf":
        manager.tf_buffer.lookup_transform.side_effect = api.TransformException("missing")
    else:
        manager.nav2.planner_is_ready.return_value = False
    manager._tick()
    manager._status.assert_called_once_with({
        "map": "WAITING_FOR_MAP", "tf": "WAITING_FOR_TF",
        "planner": "WAITING_FOR_PLANNER",
    }[missing])
    manager._begin_decision.assert_not_called()


def test_rotated_map_pose_can_record_home_and_then_plan(manager):
    manager.geometry = GridGeometry(10, 10, 0.2, 4.0, -2.0, np.pi / 2)
    x, y = grid_to_world(3, 4, manager.geometry)
    manager.tf_buffer.lookup_transform.return_value.transform.translation = NS(x=x, y=y)
    manager.home_pose = None
    manager._tick()
    assert manager.home_pose == (x, y, 0.0)
    manager._return_status.assert_called_once_with("HOME_RECORDED")
    manager._begin_decision.assert_not_called()
    manager._tick()
    manager._begin_decision.assert_called_once_with((x, y, 0.0))


def test_planner_rejection_backs_off_without_blacklisting(api, manager, monkeypatch):
    manager.busy = True
    future = Future()
    future.set_result(NS(success=False, accepted=False))
    manager._plan_done(0, future)
    assert not manager.busy and manager.retry_after == 105.0
    assert manager.plan_cursor == 0
    manager._status.assert_called_once_with("WAITING_FOR_PLANNER_ACTIVE")
    manager.get_logger().error.assert_called_once()
    manager._failure.assert_not_called()
    manager._plan_next.assert_not_called()
    monkeypatch.setattr(api, "time", NS(monotonic=lambda: 104.999))
    manager._tick()
    manager._begin_decision.assert_not_called()
    monkeypatch.setattr(api, "time", NS(monotonic=lambda: 105.0))
    manager._tick()
    manager._begin_decision.assert_called_once_with((1.5, 2.5, 0.0))


@pytest.mark.parametrize("success", [True, False])
def test_accepted_planning_results_keep_existing_behavior(manager, success):
    future = Future()
    future.set_result(NS(success=success, accepted=True, path_length_m=3.0, error="no path"))
    manager._plan_done(0, future)
    assert manager.retry_after == 0.0 and manager.plan_cursor == 1
    manager._plan_next.assert_called_once()
    if success:
        assert manager.path_distances[0] == 3.0
        manager._failure.assert_not_called()
    else:
        manager._failure.assert_called_once_with(0, "no path")


@pytest.mark.parametrize("pending", ["return", "observation"])
def test_backoff_does_not_block_return_or_observation(manager, pending):
    manager.retry_after = 105.0
    if pending == "return":
        manager.return_pending = True
    else:
        manager.observation_deadline = 99.0
    manager._tick()
    if pending == "return":
        manager._try_return_home.assert_called_once()
    else:
        manager._finish_observation.assert_called_once()
    manager._begin_decision.assert_not_called()


def test_mission_reset_clears_planner_backoff(manager):
    manager.retry_after = 105.0
    manager._mission_cb(NS(data="IDLE"))
    assert manager.retry_after == 0.0


def test_operator_cancel_without_active_goal_latches_stop(manager):
    response = NS(success=None, message="")
    result = manager._cancel_navigation(object(), response)
    assert result.success
    assert "No active navigation goal" in result.message
    assert manager.operator_cancel_requested
    assert manager.finished
    manager._status.assert_called_once_with("OPERATOR_STOPPED")


def test_operator_cancel_requests_active_goal_cancel(manager):
    request = NS(future=Future(), cancel=Mock())
    manager.active_navigation = request
    response = NS(success=None, message="")
    result = manager._cancel_navigation(object(), response)
    assert result.success
    request.cancel.assert_called_once_with()
    assert manager.operator_cancel_requested
    assert not manager.finished
    manager._status.assert_called_once_with("OPERATOR_CANCEL_REQUESTED")


def test_operator_cancel_waits_for_verified_terminal_result(manager):
    manager.operator_cancel_requested = True
    manager.busy = True
    future = Future()
    future.set_result(NS(timed_out=False, accepted=True, status=5))
    manager._nav_done(0, future)
    assert manager.finished and not manager.busy
    manager._status.assert_called_once_with("OPERATOR_STOPPED")


def test_operator_cancel_blocks_new_decisions(manager):
    manager.operator_cancel_requested = True
    manager._tick()
    manager._begin_decision.assert_not_called()
    manager.nav2.navigate_to_pose.assert_not_called()


def test_single_goal_mode_finishes_after_successful_observation(manager):
    manager.navigation_goal_count = 1
    manager.selected_index = 0
    manager.observation_deadline = 99.0
    manager._finish_observation()
    manager.memory.mark_observation_complete.assert_called_once()
    assert manager.finished
    manager._status.assert_called_once_with("SINGLE_GOAL_TEST_COMPLETE")


def test_single_goal_mode_stops_after_failed_navigation(manager):
    manager.navigation_goal_count = 1
    manager.busy = True
    future = Future()
    future.set_result(NS(success=False, error="aborted"))
    manager._nav_done(0, future)
    assert manager.finished and not manager.busy
    assert manager.failure_counts[0] == 1
    manager._status.assert_called_once_with("SINGLE_GOAL_TEST_FAILED")


def test_mission_reset_rearms_single_goal_and_operator_stop(manager):
    manager.navigation_goal_count = 1
    manager.operator_cancel_requested = True
    manager._mission_cb(NS(data="IDLE"))
    assert manager.navigation_goal_count == 0
    assert not manager.operator_cancel_requested


def test_pause_resume_does_not_rearm_single_goal(manager):
    manager.mission_state = "PAUSED"
    manager.navigation_goal_count = 1
    manager.operator_cancel_requested = True
    manager._mission_cb(NS(data="SEARCHING"))
    assert manager.navigation_goal_count == 1
    assert manager.operator_cancel_requested


@pytest.mark.parametrize("state", ["SEARCHING", "PAUSED"])
def test_memory_reset_cannot_rearm_active_mission(manager, state):
    manager.mission_state = state
    response = NS(success=None, message="")
    result = manager._reset(object(), response)
    assert not result.success
    assert "IDLE or FINISHED" in result.message
    manager.memory.reset.assert_not_called()
