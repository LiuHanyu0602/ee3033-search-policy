"""ROS-independent target selection and fresh-frame confirmation.

Does not publish /target_found or command navigation. The ROS adapter must
provide image dimensions and ROS-clock seconds matching the source header.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Observation:
    stamp: float
    frame_id: str
    class_id: int
    class_name: str
    confidence: float
    cx: float
    cy: float
    width: float
    height: float
    tracking_id: str = ''


@dataclass(frozen=True)
class DetectionFrame:
    stamp: float
    frame_id: str
    observations: tuple[Observation, ...]


class TargetConfiguration:
    """Label table must come from the deployed model, not guessed IDs."""

    def __init__(self, labels):
        self.labels = {int(k): str(v).strip().casefold() for k, v in labels.items()}
        if not self.labels or len(set(self.labels.values())) != len(self.labels):
            raise ValueError('Nonempty, unambiguous model labels required')
        self.class_id = None
        self.class_name = None
        self.threshold = None

    def configure(self, name, threshold, mission_state, detector_ready):
        if mission_state != 'WAIT_FOR_TARGET' or not detector_ready:
            raise ValueError('Configuration requires WAIT_FOR_TARGET and ready detector')
        if self.class_id is not None:
            raise ValueError('Target locked; safely reset before changing it')
        normalized = str(name).strip().casefold()
        threshold = float(threshold)
        if normalized not in self.labels.values():
            raise ValueError('Unknown model category')
        if not math.isfinite(threshold) or not 0 < threshold <= 1:
            raise ValueError('Confidence must be finite and in (0, 1]')
        self.class_id = next(k for k, v in self.labels.items() if v == normalized)
        self.class_name, self.threshold = normalized, threshold

    def reset(self, mission_state, navigation_active):
        if mission_state not in ('IDLE', 'FINISHED', 'WAIT_FOR_TARGET') or navigation_active:
            raise ValueError('Safe inactive state required for reset')
        self.class_id = self.class_name = self.threshold = None


def select_observation(msg, config, image_width, image_height):
    """Parse the user-verified yolo_msgs/DetectionArray schema.

    Image dimensions must describe the detector image, not network tensor size.
    Rotated boxes are rejected; localization currently assumes axis alignment.
    """
    if config.class_id is None:
        return None
    if image_width <= 0 or image_height <= 0:
        raise ValueError('Positive detector image dimensions required')
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    if not math.isfinite(stamp) or stamp <= 0 or not msg.header.frame_id:
        return None
    valid = []
    for d in msg.detections:
        if (d.class_id != config.class_id or
                d.class_name.strip().casefold() != config.class_name):
            continue
        b = d.bbox
        values = (d.score, b.center.position.x, b.center.position.y,
                  b.size.x, b.size.y, b.center.theta)
        if not all(math.isfinite(float(v)) for v in values):
            continue
        score, x, y, w, h, theta = map(float, values)
        if abs(theta) > 1e-6:
            continue
        valid.append(Observation(stamp, msg.header.frame_id, d.class_id,
                                 config.class_name, score, x, y, w, h, str(d.id)))
    return select_frame_observation(
        DetectionFrame(stamp, msg.header.frame_id, tuple(valid)), config,
        image_width, image_height)


def select_frame_observation(frame, config, image_width, image_height):
    """Select from normalized observations without a detector message dependency."""
    if image_width <= 0 or image_height <= 0:
        raise ValueError('Positive detector image dimensions required')
    if config.class_id is None:
        return None
    valid = []
    for o in frame.observations:
        if o.class_id != config.class_id or o.class_name.strip().casefold() != config.class_name:
            continue
        if (not all(math.isfinite(v) for v in
                    (o.confidence, o.cx, o.cy, o.width, o.height)) or
                not config.threshold <= o.confidence <= 1 or o.width <= 0 or o.height <= 0 or
                o.cx-o.width/2 < 0 or o.cy-o.height/2 < 0 or
                o.cx+o.width/2 > image_width or o.cy+o.height/2 > image_height):
            continue
        valid.append(o)
    return min(valid, key=lambda o: (-o.confidence, o.cx, o.cy, o.tracking_id)) if valid else None


def intersection_over_union(a, b):
    w = max(0., min(a.cx+a.width/2, b.cx+b.width/2)-max(a.cx-a.width/2, b.cx-b.width/2))
    h = max(0., min(a.cy+a.height/2, b.cy+b.height/2)-max(a.cy-a.height/2, b.cy-b.height/2))
    intersection = w*h
    return intersection / (a.width*a.height+b.width*b.height-intersection)


class FrameConfirmation:
    """N distinct increasing source stamps; IoU continuity; latched result.

    Call tick(now) periodically even if no detections arrive. Time is source
    ROS time, not time.monotonic(). Reset on mission reset or clock changes.
    """

    def __init__(self, required_frames=3, max_age_s=.5, max_gap_s=.4, minimum_iou=.2):
        if (not isinstance(required_frames, int) or required_frames < 1 or
                not all(math.isfinite(x) for x in (max_age_s, max_gap_s, minimum_iou)) or
                min(max_age_s, max_gap_s) <= 0 or not 0 <= minimum_iou <= 1):
            raise ValueError('Invalid confirmation configuration')
        self.required = required_frames
        self.max_age, self.max_gap, self.minimum_iou = max_age_s, max_gap_s, minimum_iou
        self.reset()

    def reset(self):
        self.count, self.previous, self.last_stamp, self.confirmed = 0, None, None, False

    def tick(self, now):
        if not self.confirmed and self.previous is not None:
            age = now - self.previous.stamp
            if not math.isfinite(now) or age < 0 or age > min(self.max_age, self.max_gap):
                self.count, self.previous = 0, None

    def update(self, observation, now):
        if self.confirmed:
            return True
        self.tick(now)
        if observation is None:
            self.count, self.previous = 0, None
            return False
        o = observation
        if not math.isfinite(now) or not 0 <= now-o.stamp <= self.max_age:
            self.count, self.previous = 0, None
            return False
        if self.last_stamp is not None and o.stamp <= self.last_stamp:
            return False
        self.last_stamp = o.stamp
        p = self.previous
        consistent = (p is not None and o.stamp-p.stamp <= self.max_gap and
                      o.frame_id == p.frame_id and o.class_id == p.class_id and
                      (not o.tracking_id or not p.tracking_id or o.tracking_id == p.tracking_id) and
                      intersection_over_union(o, p) >= self.minimum_iou)
        self.count = self.count+1 if consistent else 1
        self.previous = o
        self.confirmed = self.count >= self.required
        return self.confirmed
