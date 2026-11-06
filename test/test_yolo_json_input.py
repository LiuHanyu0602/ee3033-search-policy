import json
from types import SimpleNamespace as NS

import pytest

from ee3033_search_policy.yolo_adapter_session import AdapterSession
from ee3033_search_policy.yolo_json_input import NcnnJsonInput


LIVE_SAMPLE = {
    'timestamp': {'sec': 1793770649, 'nanosec': 722597000},
    'detections': [
        {'class_id': 39, 'class_name': 'bottle', 'confidence': .88330078125,
         'bbox': [240.875, 16.5, 365.125, 311.0]},
        {'class_id': 0, 'class_name': 'person', 'confidence': .64306640625,
         'bbox': [.125, 25.59375, 90.875, 137.40625]},
    ],
}
LIVE_SAMPLE_2 = {
    'timestamp': {'sec': 1793770661, 'nanosec': 58641000},
    'detections': [
        {'class_id': 39, 'class_name': 'bottle', 'confidence': .79248046875,
         'bbox': [312.125, 13., 433.875, 297.]},
        {'class_id': 0, 'class_name': 'person', 'confidence': .64404296875,
         'bbox': [52.34375, 15.3125, 166.15625, 111.6875]},
    ],
}


def image(sample=LIVE_SAMPLE):
    return NS(header=NS(stamp=NS(**sample['timestamp']), frame_id='default_cam'),
              width=640, height=480, encoding='rgb8')


def camera_info(sample=LIVE_SAMPLE):
    return NS(header=NS(stamp=NS(**sample['timestamp']), frame_id='default_cam'),
              width=640, height=480)


def source_time(sample=LIVE_SAMPLE):
    return sample['timestamp']['sec'] + sample['timestamp']['nanosec']/1e9


@pytest.fixture
def source():
    source = NcnnJsonInput()
    source.record_image(image(), 100.)
    return source


@pytest.mark.parametrize('sample,expected', [
    (LIVE_SAMPLE, (303., 163.75, 124.25, 294.5)),
    (LIVE_SAMPLE_2, (373., 155., 121.75, 284.)),
])
def test_real_pi_samples_preserve_geometry_timestamp_and_frame(sample, expected):
    source = NcnnJsonInput()
    source.record_image(image(sample), 100.)
    frame, metadata = source.match(json.dumps(sample), source_time(sample)+.1, 100.1, .5)
    o = frame.observations[0]
    assert (o.cx, o.cy, o.width, o.height) == expected
    assert o.frame_id == frame.frame_id == 'default_cam'
    assert o.stamp == frame.stamp == source_time(sample)
    assert o.confidence == sample['detections'][0]['confidence']
    assert o.tracking_id == ''
    assert metadata.stamp_ns == (sample['timestamp']['sec']*1_000_000_000 +
                                 sample['timestamp']['nanosec'])
    assert len(frame.observations) == 2
    assert (metadata.width, metadata.height) == (640, 480)


def test_no_matching_image_never_borrows_latest_frame(source):
    sample = json.loads(json.dumps(LIVE_SAMPLE))
    sample['timestamp']['nanosec'] += 1
    with pytest.raises(ValueError, match='exactly matching'):
        source.match(json.dumps(sample), source_time()+.1, 100.1, .5)


def test_matching_camera_info_supplies_exact_source_metadata():
    source = NcnnJsonInput()
    source.record_camera_info(camera_info(), 100.)
    frame, metadata = source.match(
        json.dumps(LIVE_SAMPLE), source_time()+.1, 100.1, .5)
    assert frame.frame_id == metadata.frame_id == 'default_cam'
    assert metadata.stamp_ns == (
        LIVE_SAMPLE['timestamp']['sec']*1_000_000_000
        + LIVE_SAMPLE['timestamp']['nanosec'])
    assert (metadata.width, metadata.height) == (640, 480)


@pytest.mark.parametrize('ros_now,steady_now', [
    (source_time()+.6, 100.1), (source_time()-.1, 100.1),
    (source_time()+.1, 100.6), (source_time()+.1, 99.9),
])
def test_stale_future_or_stale_image_receipt_rejected(source, ros_now, steady_now):
    with pytest.raises(ValueError):
        source.match(json.dumps(LIVE_SAMPLE), ros_now, steady_now, .5)


@pytest.mark.parametrize('text', [
    '{', '[]', '{}', '{"timestamp":null}', '{"timestamp":{"sec":1.5,"nanosec":0}}',
    '{"timestamp":{"sec":true,"nanosec":0}}',
    '{"timestamp":{"sec":0,"nanosec":0}}',
    '{"timestamp":{"sec":1,"nanosec":1000000000}}',
])
def test_invalid_json_and_timestamps_rejected(source, text):
    with pytest.raises(ValueError):
        source.match(text, source_time()+.1, 100.1, .5)


@pytest.mark.parametrize('field,value', [
    ('class_id', True), ('class_id', 39.5), ('class_name', ''),
    ('confidence', float('nan')), ('confidence', float('inf')),
    ('confidence', '0.9'), ('confidence', True), ('confidence', 1.1),
    ('bbox', [1, 2, 3]), ('bbox', [-1, 0, 20, 20]),
    ('bbox', [10, 10, 10, 20]), ('bbox', [10, 10, 641, 20]),
    ('bbox', [10, 10, 20, float('nan')]),
])
def test_invalid_detection_payload_rejected(source, field, value):
    sample = json.loads(json.dumps(LIVE_SAMPLE))
    sample['detections'][0][field] = value
    with pytest.raises(ValueError):
        source.match(json.dumps(sample), source_time()+.1, 100.1, .5)


@pytest.mark.parametrize('detections', [None, {}, [None]])
def test_invalid_detection_array_rejected(source, detections):
    sample = dict(LIVE_SAMPLE, detections=detections)
    with pytest.raises(ValueError):
        source.match(json.dumps(sample), source_time()+.1, 100.1, .5)


def test_metadata_cache_bounded_and_clearable():
    source = NcnnJsonInput()
    for i in range(100):
        msg = image()
        msg.header.stamp.nanosec = i
        source.record_image(msg, 100.)
    assert len(source.images) == 64
    assert all(not hasattr(m, 'data') for m in source.images.values())
    source.clear()
    assert not source.images


@pytest.mark.parametrize('field,value', [
    ('encoding', 'mono8'), ('width', 0), ('height', 0),
])
def test_invalid_image_metadata_rejected(field, value):
    msg = image()
    setattr(msg, field, value)
    with pytest.raises(ValueError):
        NcnnJsonInput().record_image(msg, 100.)


def test_json_session_configuration_confirmation_duplicates_and_empty_frames(source):
    session = AdapterSession({39: 'bottle', 0: 'person'})
    session.set_state('WAIT_FOR_TARGET')
    frame, metadata = source.match(json.dumps(LIVE_SAMPLE), source_time(), 100., .5)
    assert session.ingest_frame(frame, 640, 480, source_time(), 100.) is None
    session.configure('bottle', .65, source_time(), 100.)
    session.set_state('SEARCHING')
    for i in range(1, 4):
        sample = json.loads(json.dumps(LIVE_SAMPLE))
        sample['timestamp']['nanosec'] += i*30_000_000
        source.record_image(image(sample), 100.+i*.03)
        frame, metadata = source.match(
            json.dumps(sample), source_time(sample), 100.+i*.03, .5)
        o = session.ingest_frame(frame, 640, 480, source_time(sample), 100.+i*.03)
        assert o.class_name == 'bottle'
        count = session.frames.count
        assert session.ingest_frame(frame, 640, 480, source_time(sample), 100.+i*.03) is None
        assert session.frames.count == count
    assert session.frames.confirmed
    session.frames.reset()
    sample['timestamp']['nanosec'] += 30_000_000
    sample['detections'] = []
    source.record_image(image(sample), 100.12)
    frame, metadata = source.match(json.dumps(sample), source_time(sample), 100.12, .5)
    assert session.ingest_frame(frame, 640, 480, source_time(sample), 100.12) is None
    assert session.frames.count == 0
    assert session.ready(source_time(sample), 100.12)
