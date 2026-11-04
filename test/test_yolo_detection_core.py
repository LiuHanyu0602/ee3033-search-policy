from dataclasses import replace
from types import SimpleNamespace as NS

import pytest

from ee3033_search_policy.yolo_detection_core import (
    FrameConfirmation, Observation, TargetConfiguration, select_observation,
)


def config():
    c = TargetConfiguration({39: 'bottle', 41: 'cup'})
    c.configure(' Bottle ', .65, 'WAIT_FOR_TARGET', True)
    return c


def obs(t=10., **kwargs):
    return replace(Observation(t, 'camera_optical', 39, 'bottle', .9, 100, 100, 40, 60), **kwargs)


def message(**kwargs):
    fields = dict(class_id=39, class_name='bottle', score=.9, id='',
                  bbox=NS(center=NS(position=NS(x=100., y=100.), theta=0.),
                          size=NS(x=40., y=60.)))
    fields.update(kwargs)
    return NS(header=NS(stamp=NS(sec=10, nanosec=0), frame_id='camera_optical'),
              detections=[NS(**fields)])


def test_configuration_and_lock():
    c = config()
    assert c.class_id == 39
    with pytest.raises(ValueError):
        c.configure('cup', .7, 'WAIT_FOR_TARGET', True)
    with pytest.raises(ValueError):
        c.reset('SEARCHING', False)
    c.reset('IDLE', False)
    assert c.class_id is None


@pytest.mark.parametrize('name,threshold,state,ready', [
    ('botle', .65, 'WAIT_FOR_TARGET', True),
    ('bottle', float('nan'), 'WAIT_FOR_TARGET', True),
    ('bottle', .65, 'SEARCHING', True),
    ('bottle', .65, 'WAIT_FOR_TARGET', False),
])
def test_bad_configuration(name, threshold, state, ready):
    with pytest.raises(ValueError):
        TargetConfiguration({39: 'bottle'}).configure(name, threshold, state, ready)


def test_verified_schema():
    o = select_observation(message(), config(), 640, 480)
    assert (o.cx, o.cy, o.width, o.stamp) == (100, 100, 40, 10)


@pytest.mark.parametrize('kwargs', [dict(score=.2), dict(score=float('nan')),
                                      dict(class_id=41), dict(class_name='cup')])
def test_invalid_detection(kwargs):
    assert select_observation(message(**kwargs), config(), 640, 480) is None


def test_bbox_bounds_and_rotation():
    m = message()
    m.detections[0].bbox.size.x = 1000
    assert select_observation(m, config(), 640, 480) is None
    m = message()
    m.detections[0].bbox.center.theta = .1
    assert select_observation(m, config(), 640, 480) is None


def test_three_distinct_frames_and_latch():
    f = FrameConfirmation()
    assert not f.update(obs(), 10.)
    assert not f.update(obs(), 10.01)
    assert f.count == 1
    assert not f.update(obs(10.1), 10.1)
    assert f.update(obs(10.2), 10.2)
    assert f.update(None, 20)
    f.reset()
    assert not f.confirmed


@pytest.mark.parametrize('t,now', [(10., 11.), (11., 10.)])
def test_stale_or_future(t, now):
    f = FrameConfirmation()
    assert not f.update(obs(t), now)
    assert f.count == 0


def test_gap_miss_and_spatial_inconsistency():
    f = FrameConfirmation()
    f.update(obs(), 10)
    f.update(obs(10.1, cx=400), 10.1)
    assert f.count == 1
    f.update(None, 10.2)
    assert f.count == 0
    f.update(obs(11), 11)
    f.tick(11.5)
    assert f.count == 0


def test_class_frame_tracking_change():
    for changes in (dict(class_id=41), dict(frame_id='other'), dict(tracking_id='b')):
        f = FrameConfirmation()
        f.update(obs(tracking_id='a'), 10)
        f.update(obs(10.1, **changes), 10.1)
        assert f.count == 1
