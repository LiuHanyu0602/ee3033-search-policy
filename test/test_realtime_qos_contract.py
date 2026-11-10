import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1] / "ee3033_search_policy"


def tree(name):
    return ast.parse((ROOT / name).read_text())


def calls(source, method):
    return [
        node for node in ast.walk(source)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == method
    ]


def topic(call):
    value = call.args[1]
    return value.value if isinstance(value, ast.Constant) else None


def qos_name(call):
    value = call.args[-1]
    return value.id if isinstance(value, ast.Name) else None


def assert_latest_profile(source, name):
    assignment = next(
        node for node in ast.walk(source)
        if isinstance(node, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == name
                for target in node.targets)
    )
    profile = assignment.value
    keywords = {item.arg: item.value for item in profile.keywords}
    assert keywords["depth"].value == 1
    reliability = keywords["reliability"]
    assert isinstance(reliability, ast.Attribute)
    assert reliability.attr == "BEST_EFFORT"


def test_vision_streams_use_latest_sample_qos():
    adapter = tree("yolo_detection_adapter_node.py")
    assert_latest_profile(adapter, "sensors")
    observation = next(
        call for call in calls(adapter, "create_publisher")
        if topic(call) == "/target/observation"
    )
    json_input = next(
        call for call in calls(adapter, "create_subscription")
        if isinstance(call.args[2], ast.Attribute)
        and call.args[2].attr == "_detect_json"
    )
    assert qos_name(observation) == qos_name(json_input) == "sensors"

    localizer = tree("stamped_target_localizer_node.py")
    assert_latest_profile(localizer, "realtime")
    for method, name in (
        ("create_publisher", "/target/estimate_candidate"),
        ("create_subscription", "/target/observation"),
    ):
        call = next(item for item in calls(localizer, method) if topic(item) == name)
        assert qos_name(call) == "realtime"


def test_motion_and_heartbeat_streams_use_latest_sample_qos():
    coordinator = tree("target_mission_coordinator.py")
    assert_latest_profile(coordinator, "realtime")
    for method, name in (
        ("create_publisher", "/mission/lease"),
        ("create_subscription", "/search/runtime_state"),
        ("create_subscription", "/target/observation"),
        ("create_subscription", "/target/estimate_candidate"),
    ):
        call = next(item for item in calls(coordinator, method) if topic(item) == name)
        assert qos_name(call) == "realtime"

    manager = tree("guarded_search_manager.py")
    assert_latest_profile(manager, "realtime")
    for method, name in (
        ("create_publisher", "/search/runtime_state"),
        ("create_subscription", "/mission/lease"),
    ):
        call = next(item for item in calls(manager, method) if topic(item) == name)
        assert qos_name(call) == "realtime"


def test_euler_input_keeps_only_latest_sample():
    source = tree("euler_camera_tf_node.py")
    assert_latest_profile(source, "qos")
