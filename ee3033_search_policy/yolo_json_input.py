"""Match the deployed NCNN JSON schema to exact source-image metadata."""

from collections import OrderedDict
from dataclasses import dataclass
import json
import math

from .yolo_detection_core import DetectionFrame, Observation


def stamp_nanoseconds(sec, nanosec):
    if (type(sec) is not int or type(nanosec) is not int or sec < 0 or
            not 0 <= nanosec < 1_000_000_000 or sec+nanosec == 0):
        raise ValueError('Expected positive source timestamp with integer sec/nanosec')
    return sec*1_000_000_000 + nanosec


@dataclass(frozen=True)
class ImageMetadata:
    stamp_ns: int
    frame_id: str
    width: int
    height: int
    received_at: float


class NcnnJsonInput:
    def __init__(self):
        self.images = OrderedDict()

    def clear(self):
        self.images.clear()

    def record_image(self, image, steady_now):
        stamp = stamp_nanoseconds(image.header.stamp.sec, image.header.stamp.nanosec)
        if (not image.header.frame_id or image.width <= 0 or image.height <= 0 or
                image.encoding not in ('rgb8', 'bgr8')):
            raise ValueError('Expected a named, positive-size rgb8/bgr8 source image')
        # Retain metadata only, never image buffers; bounded to about two seconds at 30 Hz.
        self.images[stamp] = ImageMetadata(
            stamp, image.header.frame_id, image.width, image.height, steady_now)
        self.images.move_to_end(stamp)
        while len(self.images) > 64:
            self.images.popitem(last=False)

    def match(self, text, ros_now, steady_now, max_age):
        data = json.loads(text)
        if not isinstance(data, dict) or not isinstance(data.get('timestamp'), dict):
            raise ValueError('Expected JSON object with source timestamp')
        timestamp = data['timestamp']
        stamp = stamp_nanoseconds(timestamp.get('sec'), timestamp.get('nanosec'))
        source_stamp = timestamp['sec'] + timestamp['nanosec']*1e-9
        if not 0 <= ros_now-source_stamp <= max_age:
            raise ValueError('NCNN source timestamp is stale or in the future')
        image = self.images.get(stamp)
        if image is None:
            raise ValueError('No source Image metadata with exactly matching NCNN timestamp')
        if not 0 <= steady_now-image.received_at <= max_age:
            raise ValueError('Matching source Image metadata is stale')
        detections = data.get('detections')
        if not isinstance(detections, list):
            raise ValueError('Expected detections array, including [] for no objects')
        observations = []
        for detection in detections:
            if not isinstance(detection, dict):
                raise ValueError('Expected detection object')
            class_id, name = detection.get('class_id'), detection.get('class_name')
            confidence, bbox = detection.get('confidence'), detection.get('bbox')
            if (type(class_id) is not int or class_id < 0 or
                    not isinstance(name, str) or not name.strip() or
                    not isinstance(bbox, list) or len(bbox) != 4):
                raise ValueError('Invalid NCNN class or xyxy bbox')
            if not all(type(v) in (int, float) and math.isfinite(v)
                       for v in [confidence, *bbox]):
                raise ValueError('Confidence and bbox must be finite JSON numbers')
            x1, y1, x2, y2 = bbox
            if (not 0 <= confidence <= 1 or
                    not 0 <= x1 < x2 <= image.width or
                    not 0 <= y1 < y2 <= image.height):
                raise ValueError('Invalid confidence or bbox outside source image')
            observations.append(Observation(
                source_stamp, image.frame_id, class_id, name.strip().casefold(),
                float(confidence), (x1+x2)/2, (y1+y2)/2, x2-x1, y2-y1))
        return DetectionFrame(source_stamp, image.frame_id, tuple(observations)), image
