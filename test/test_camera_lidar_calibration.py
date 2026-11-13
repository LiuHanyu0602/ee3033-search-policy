import math

import pytest

from ee3033_search_policy.camera_lidar_calibration_core import (
    CalibrationSample,
    summarize_samples,
)


def sample(camera_bearing=.1, lidar_bearing=.05,
           camera_range=2., lidar_range=1.8,
           camera_depression=.2, required_depression=.15,
           confidence=.9):
    return CalibrationSample(
        camera_bearing, lidar_bearing,
        camera_range, lidar_range,
        camera_depression, required_depression,
        confidence,
    )


def test_summary_reports_physical_mount_adjustments():
    result = summarize_samples([sample(), sample(camera_range=2.1)])
    assert result.sample_count == 2
    assert math.degrees(result.horizontal_adjustment_rad) == pytest.approx(
        math.degrees(.05))
    assert result.vertical_adjustment_rad == pytest.approx(.05)
    assert result.range_error_m == pytest.approx(.25)
    assert result.confidence == pytest.approx(.9)


def test_summary_wraps_horizontal_error_at_pi():
    result = summarize_samples([
        sample(camera_bearing=-math.pi+.02, lidar_bearing=math.pi-.01),
    ])
    assert result.horizontal_adjustment_rad == pytest.approx(.03)


def test_invalid_sample_is_rejected():
    with pytest.raises(ValueError):
        sample(camera_range=float('nan'))


def test_new_entry_is_isolated_from_proven_real_workflow():
    from pathlib import Path

    root = Path(__file__).resolve().parents[1]
    calibration_launch = (
        root/'launch'/'camera_lidar_calibration_support.launch.py').read_text()
    calibration_script = (
        root/'scripts'/'run_camera_lidar_calibration.sh').read_text()
    real_script = (root/'scripts'/'run_real_robot_workflow.sh').read_text()
    assert 'camera_lidar_calibration' in calibration_script
    assert 'navigation_launch' not in calibration_launch
    assert 'guarded_search_manager' not in calibration_launch
    assert 'target_mission_coordinator' not in calibration_launch
    assert 'real_robot_workflow' in real_script
    assert 'camera_lidar_calibration' not in real_script
    calibration_source = (
        root/'ee3033_search_policy'/'camera_lidar_calibration.py').read_text()
    assert "self.create_timer(.2, self._publish_desired_state)" in calibration_source
    assert 'adapter={self.adapter_status' in calibration_source
