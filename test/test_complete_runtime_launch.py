from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_validation_launch_isolates_high_rate_project_nodes():
    launch = (ROOT / "launch" / "yolo_complete_validation.launch.py").read_text()

    assert "ee3033_search_policy.complete_runtime_node" not in launch
    assert "DeclareLaunchArgument('single_goal_mode', default_value='true')" in launch
    assert "DeclareLaunchArgument('max_navigation_goals', default_value='1')" in launch
    assert "['single_goal_mode:=', single_goal]" in launch
    assert "['max_navigation_goals:=', max_goals]" in launch
    assert "return_home_enabled:=false" in launch
    for module in (
        "euler_camera_tf_node",
        "continuous_observation_adapter",
        "stamped_target_localizer_node",
        "target_mission_coordinator",
        "guarded_search_manager",
    ):
        assert module in launch
    assert "'ee3033_search_policy.' + module" in launch


def test_removed_combined_runtime_cannot_regress_executor_load():
    assert not (
        ROOT / "ee3033_search_policy" / "complete_runtime_node.py"
    ).exists()
