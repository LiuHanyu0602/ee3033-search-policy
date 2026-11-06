from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_validation_launch_uses_one_project_runtime_process():
    launch = (ROOT / "launch" / "yolo_complete_validation.launch.py").read_text()

    assert "ee3033_search_policy.complete_runtime_node" in launch
    assert "single_goal_mode:=true" in launch
    assert "max_navigation_goals:=1" in launch
    assert "return_home_enabled:=false" in launch
    for module in (
        "euler_camera_tf_node",
        "continuous_observation_adapter",
        "stamped_target_localizer_node",
        "target_mission_coordinator",
        "guarded_search_manager",
    ):
        assert f"ee3033_search_policy.{module}" not in launch


def test_complete_runtime_has_bounded_executor_and_required_nodes():
    runtime = (
        ROOT / "ee3033_search_policy" / "complete_runtime_node.py"
    ).read_text()

    assert "SingleThreadedExecutor()" in runtime
    for node_type in (
        "EulerCameraTf",
        "ContinuousObservationAdapter",
        "StampedTargetLocalizer",
        "TargetMissionCoordinator",
        "GuardedSearchManager",
    ):
        assert node_type in runtime
