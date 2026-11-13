from pathlib import Path

import pytest

from ee3033_search_policy.workflow_support import (
    PlanarPose,
    angular_error,
    clean_isolated_occupied_pixels,
    saved_map_files,
    planar_pose_error,
    validate_saved_map_thresholds,
    validate_new_map_prefix,
)


ROOT = Path(__file__).resolve().parents[1]


def test_map_prefix_must_be_new_absolute_prefix(tmp_path):
    prefix = tmp_path / 'map_new'
    assert validate_new_map_prefix(prefix) == prefix
    with pytest.raises(ValueError, match='must not include'):
        validate_new_map_prefix(prefix.with_suffix('.yaml'))
    prefix.with_suffix('.pgm').write_bytes(b'old')
    with pytest.raises(ValueError, match='Refusing to overwrite'):
        validate_new_map_prefix(prefix)


def test_saved_map_validation_requires_matching_image(tmp_path):
    prefix = tmp_path / 'map_new'
    prefix.with_suffix('.pgm').write_bytes(b'P5\n1 1\n255\n\0')
    prefix.with_suffix('.yaml').write_text(
        'image: map_new.pgm\nresolution: 0.05\n', encoding='utf-8')
    assert saved_map_files(prefix) == (
        prefix.with_suffix('.yaml'), prefix.with_suffix('.pgm'))


def test_pose_and_angle_validation():
    assert PlanarPose(1.0, 2.0, 3.0).x == 1.0
    assert angular_error(3.13, -3.13) < 0.03
    with pytest.raises(ValueError):
        PlanarPose(float('nan'), 0.0, 0.0)
    distance, yaw = planar_pose_error(
        PlanarPose(3., 4., -3.13), PlanarPose(0., 0., 3.13))
    assert distance == pytest.approx(5.)
    assert yaw < .03


def write_test_map(tmp_path, pixels):
    prefix = tmp_path/'map_filter'
    prefix.with_suffix('.pgm').write_bytes(
        b'P5\n# test\n9 9\n255\n'+bytes(pixels))
    prefix.with_suffix('.yaml').write_text(
        'image: map_filter.pgm\nmode: trinary\nresolution: 0.05\n'
        'origin: [0, 0, 0]\nnegate: 0\noccupied_thresh: 0.65\n'
        'free_thresh: 0.196\n', encoding='utf-8')
    return prefix


def test_isolated_occupied_pixel_is_removed_and_raw_map_retained(tmp_path):
    pixels = [255]*81
    pixels[40] = 0
    prefix = write_test_map(tmp_path, pixels)
    result = clean_isolated_occupied_pixels(
        prefix.with_suffix('.yaml'), prefix.with_suffix('.pgm'))
    assert result.removed == 1
    assert result.raw_path.read_bytes()[-81:][40] == 0
    assert prefix.with_suffix('.pgm').read_bytes()[-81:][40] == 255
    assert result.raw_sha256 != result.cleaned_sha256


@pytest.mark.parametrize('neighbor_value', [0, 127, 205])
def test_connected_or_unknown_neighbor_preserves_occupied_pixel(
        tmp_path, neighbor_value):
    pixels = [255]*81
    pixels[40] = 0
    pixels[41] = neighbor_value
    prefix = write_test_map(tmp_path, pixels)
    result = clean_isolated_occupied_pixels(
        prefix.with_suffix('.yaml'), prefix.with_suffix('.pgm'))
    assert result.removed == 0
    assert prefix.with_suffix('.pgm').read_bytes()[-81:][40] == 0


def test_obstacle_two_cells_away_is_outside_three_by_three_filter(tmp_path):
    pixels = [255]*81
    pixels[40] = 0
    pixels[42] = 0
    prefix = write_test_map(tmp_path, pixels)
    result = clean_isolated_occupied_pixels(
        prefix.with_suffix('.yaml'), prefix.with_suffix('.pgm'))
    assert result.removed == 2
    assert prefix.with_suffix('.pgm').read_bytes()[-81:][40] == 255
    assert prefix.with_suffix('.pgm').read_bytes()[-81:][42] == 255


def test_saved_map_threshold_validation_preserves_unknown_gray(tmp_path):
    prefix = write_test_map(tmp_path, [205]*81)
    assert validate_saved_map_thresholds(prefix.with_suffix('.yaml')) == (
        0, 0.65, 0.196)
    prefix.with_suffix('.yaml').write_text(
        prefix.with_suffix('.yaml').read_text().replace('0.196', '0.25'),
        encoding='utf-8')
    with pytest.raises(ValueError, match='does not preserve unknown'):
        validate_saved_map_thresholds(prefix.with_suffix('.yaml'))


def test_workflow_contains_complete_owned_process_sequence():
    workflow = (ROOT/'ee3033_search_policy'/'real_robot_workflow.py').read_text()
    wrapper = (ROOT/'scripts'/'run_real_robot_workflow.sh').read_text()
    config = (ROOT/'config'/'real_robot_workflow.yaml').read_text()

    for required in (
        'balance_slam_bringup.launch.py', 'map_saver_cli',
        'clean_isolated_occupied_pixels',
        'balance_known_map_waypoint_bringup.launch.py', '/initialpose',
        '/camera/image_raw', 'yolo_ncnn_node.py',
        'yolo_complete_validation.launch.py', '/target/set_class',
        '/mission/start_search', '/navigate_to_pose',
    ):
        assert required in workflow
    assert 'shell=True' not in workflow
    assert '--map-prefix' in workflow
    assert '--navigation-enabled' in workflow
    assert 'usb_cam_timestamp_test.hZhIXi' in wrapper
    assert 'required_frames: 1' in config
    assert 'max_age_s: 1.5' in config
    assert 'maximum_age_s: 0.5' in config
    assert 'detector_fault_grace_s: 0.75' in config
    assert 'range_mode: nearest_lidar_then_ground' in config
    assert "'free_thresh_default:=0.196'" in workflow
    assert 'validate_saved_map_thresholds(yaml_path)' in workflow
    assert 'lidar_ground_gate_m: 0.20' in config
    assert 'scan_sync_tolerance_s: 0.12' in config
    assert 'target_alignment_enabled: true' in config
    assert "'/target/localization_status'" in workflow
    assert "'/target/estimate_candidate'" in workflow
    assert 'localization candidate: method=' in workflow
    assert 'mapping start recorded:' in workflow
    assert 'mapping return check:' in workflow
    assert 'target coordinate:' in workflow
    assert 'start coordinate:' in workflow
    assert 'start-to-target Euclidean distance:' in workflow
    assert 'math.hypot(' in workflow
    assert 'consecutive >= 3' in workflow
    assert "'/amcl_pose'" in workflow
    assert 'amcl_pose_history' in workflow
    assert 'self.lifecycle_clients = {}' in workflow
    assert 'self.lifecycle_clients.get(node_name)' in workflow
