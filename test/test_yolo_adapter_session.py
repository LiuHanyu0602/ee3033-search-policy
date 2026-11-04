from types import SimpleNamespace as NS
import pytest
from ee3033_search_policy.yolo_adapter_session import AdapterSession


def msg(t, name='bottle', score=.9):
    return NS(header=NS(stamp=NS(sec=int(t), nanosec=round((t-int(t))*1e9)), frame_id='optical'),
              detections=[NS(class_id=39, class_name=name, score=score, id='',
              bbox=NS(center=NS(position=NS(x=100., y=100.), theta=0.), size=NS(x=40., y=60.)))])


def session():
    s = AdapterSession({39: 'bottle'})
    s.set_state('WAIT_FOR_TARGET')
    s.ingest(msg(10), 640, 480, 10, 100)
    s.configure('bottle', .65, 10, 100)
    return s


def test_no_configuration_without_source():
    s = AdapterSession({39: 'bottle'})
    s.set_state('WAIT_FOR_TARGET')
    with pytest.raises(ValueError):
        s.configure('bottle', .65, 10, 100)


def test_configuration_locked_and_no_confirmation_before_search():
    s = session()
    assert s.ingest(msg(10.1), 640, 480, 10.1, 100.1) is None
    with pytest.raises(ValueError):
        s.configure('bottle', .8, 10.1, 100.1)


def test_confirm_and_episode_reset():
    s = session()
    episode = s.episode_id
    s.set_state('SEARCHING')
    for t in (10.1, 10.2, 10.3):
        assert s.ingest(msg(t), 640, 480, t, 100+t-10) is not None
    assert s.frames.confirmed
    s.set_state('IDLE')
    assert not s.episode_id and not s.frames.confirmed
    assert episode


def test_pause_resets_partial_sequence():
    s = session()
    s.set_state('SEARCHING')
    s.ingest(msg(10.1), 640, 480, 10.1, 100.1)
    s.set_state('PAUSED')
    assert s.frames.count == 0


def test_replayed_source_does_not_refresh_readiness():
    s = session()
    s.ingest(msg(10), 640, 480, 10.1, 100.1)
    assert s.last_receipt == 100
    assert not s.ready(10.6, 100.6)


def test_clock_rewind_invalidates_episode():
    s = session()
    s.tick(10.1, 100.1)
    s.tick(9, 100.2)
    assert s.episode_id == ''
