import math
import importlib.util
from pathlib import Path
import sys
from types import ModuleType
from types import SimpleNamespace as NS
from unittest.mock import Mock

import numpy as np
import pytest

from ee3033_search_policy.yolo_bearing import RawCameraCalibration


CALIBRATION = Path(__file__).resolve().parents[1] / 'results/updated_calibration/calibration.npz'


@pytest.fixture
def calibration():
    return RawCameraCalibration.load(CALIBRATION)


def camera_info(calibration):
    c = calibration.camera
    return NS(width=calibration.width, height=calibration.height,
              header=NS(frame_id='camera_optical'), distortion_model='plumb_bob',
              k=[c.fx, 0, c.cx, 0, c.fy, c.cy, 0, 0, 1], d=list(c.distortion),
              binning_x=0, binning_y=0,
              roi=NS(x_offset=0, y_offset=0, width=0, height=0, do_rectify=False))


def distort(camera, x, y):
    k1, k2, p1, p2, k3 = camera.distortion
    r2 = x*x+y*y
    radial = 1+k1*r2+k2*r2*r2+k3*r2*r2*r2
    return (camera.fx*(x*radial+2*p1*x*y+p2*(r2+2*x*x))+camera.cx,
            camera.fy*(y*radial+p1*(r2+2*y*y)+2*p2*x*y)+camera.cy)


def test_updated_calibration_and_principal_point(calibration):
    assert (calibration.width, calibration.height) == (640, 480)
    assert calibration.camera.fx == pytest.approx(604.5025053922062)
    assert calibration.bearing(calibration.camera.cx, calibration.camera.cy) == 0
    calibration.validate_camera_info(camera_info(calibration))


@pytest.mark.parametrize('x,y', [
    (-.4, -.25), (-.2, .2), (0, -.3), (0, .3), (.2, -.2), (.4, .25),
])
def test_recovers_known_optical_bearing_after_distortion(calibration, x, y):
    u, v = distort(calibration.camera, x, y)
    assert calibration.bearing(u, v) == pytest.approx(math.atan2(x, 1), abs=1e-7)


@pytest.mark.parametrize('u,v', [(0, 0), (0, 479), (600, 240)])
def test_supported_edge_pixels_converge(calibration, u, v):
    assert math.isfinite(calibration.bearing(u, v))


@pytest.mark.parametrize('u,v', [(639, 0), (639, 479)])
def test_current_calibration_nonconvergent_corners_are_rejected(calibration, u, v):
    with pytest.raises(ValueError, match='did not converge'):
        calibration.bearing(u, v)


@pytest.mark.parametrize('u,v', [
    (float('nan'), 100), (100, float('inf')), (-1, 100), (640, 100), (100, 480),
])
def test_invalid_centres_fail_explicitly(calibration, u, v):
    with pytest.raises(ValueError):
        calibration.bearing(u, v)


@pytest.mark.parametrize('field,value', [
    ('width', 320), ('height', 320), ('distortion_model', 'equidistant'),
    ('k', [1]*9), ('d', [0]*5), ('binning_x', 2),
])
def test_mismatched_geometry_rejected(calibration, field, value):
    info = camera_info(calibration)
    setattr(info, field, value)
    with pytest.raises(ValueError):
        calibration.validate_camera_info(info)


def test_cropped_or_rectified_geometry_rejected(calibration):
    info = camera_info(calibration)
    info.roi.x_offset = 10
    with pytest.raises(ValueError):
        calibration.validate_camera_info(info)
    info.roi.x_offset, info.roi.do_rectify = 0, True
    with pytest.raises(ValueError):
        calibration.validate_camera_info(info)


@pytest.mark.parametrize('key,value', [
    ('camera_matrix', np.eye(2)), ('camera_matrix', np.full((3, 3), np.nan)),
    ('dist_coeffs', np.zeros(4)), ('image_width', 0), ('image_height', 480.5),
])
def test_invalid_calibration_rejected(tmp_path, key, value):
    with np.load(CALIBRATION, allow_pickle=False) as source:
        data = {name: source[name] for name in source.files}
    data[key] = value
    path = tmp_path / 'bad.npz'
    np.savez(path, **data)
    with pytest.raises(ValueError):
        RawCameraCalibration.load(path)


@pytest.fixture
def adapter(calibration, monkeypatch):
    modules = {
        'ament_index_python.packages': {'get_package_share_directory': Mock()},
        'rclpy': {},
        'rclpy.node': {'Node': object},
        'rclpy.qos': {'QoSProfile': object, 'ReliabilityPolicy': object,
                      'DurabilityPolicy': object},
        'sensor_msgs.msg': {'CameraInfo': object},
        'std_msgs.msg': {'String': object},
        'std_srvs.srv': {'Trigger': object},
        'yolo_msgs.msg': {'DetectionArray': object},
        'ee3033_search_policy_interfaces.msg': {'TargetObservation': NS},
        'ee3033_search_policy_interfaces.srv': {'SetTargetClass': object},
    }
    for name, exports in modules.items():
        module = ModuleType(name)
        module.__dict__.update(exports)
        monkeypatch.setitem(sys.modules, name, module)
    path = CALIBRATION.parents[2] / 'ee3033_search_policy/yolo_detection_adapter_node.py'
    spec = importlib.util.spec_from_file_location(
        'ee3033_search_policy._bearing_adapter_under_test', path)
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, module)
    spec.loader.exec_module(module)
    node = module.YoloDetectionAdapter.__new__(module.YoloDetectionAdapter)
    node.calibration = calibration
    node.geometry_verified = True
    node.camera = camera_info(calibration)
    node.last_frame = ''
    node.session = Mock()
    node.session.episode_id = 'episode-1'
    node.session.frames.confirmed = True
    node._now = Mock(return_value=10.)
    node._configuration = Mock()
    node.get_logger = Mock(return_value=Mock())
    node.observations = Mock()
    return node


def test_adapter_publishes_bearing_and_preserves_observation(adapter):
    o = NS(cx=400., cy=240., width=40., height=60., class_id=39,
           class_name='bottle', confidence=.9, tracking_id='object-1')
    adapter.session.ingest.return_value = o
    header = NS(frame_id='camera_optical', stamp=NS(sec=10, nanosec=123))
    adapter._detect(NS(header=header))
    result = adapter.observations.publish.call_args.args[0]
    assert result.bearing_rad == pytest.approx(adapter.calibration.bearing(400, 240))
    assert result.bearing_rad > 0
    assert result.header is header
    assert result.episode_id == 'episode-1'
    assert (result.center_x, result.center_y, result.width, result.height) == (400, 240, 40, 60)
    assert (result.image_width, result.image_height) == (640, 480)
    assert result.tracking_id == 'object-1'
    assert result.confirmed


def test_adapter_does_not_publish_failed_undistortion(adapter):
    adapter.session.ingest.return_value = NS(cx=639., cy=0.)
    adapter._detect(NS(header=NS(frame_id='camera_optical')))
    adapter.observations.publish.assert_not_called()
    adapter.get_logger.return_value.warning.assert_called_once()
    adapter.session.frames.reset.assert_called_once()
    adapter.session.frames.update.assert_called_once_with(None, 10.)


def test_adapter_rejects_mismatched_calibration_and_clears_episode(adapter):
    info = camera_info(adapter.calibration)
    info.k[0] = 303.
    adapter._camera(info)
    assert adapter.camera is None
    assert adapter.last_frame == ''
    adapter.session.clear.assert_called_once()
    adapter._configuration.assert_called_once()
    adapter.get_logger.return_value.error.assert_called_once()
    adapter._detect(NS(header=NS(frame_id='camera_optical')))
    adapter.observations.publish.assert_not_called()


def test_adapter_accepts_matching_camera_info(adapter):
    info = camera_info(adapter.calibration)
    adapter._camera(info)
    assert adapter.camera is info
    adapter.session.clear.assert_not_called()
