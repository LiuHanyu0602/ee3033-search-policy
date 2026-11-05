"""Bounded balancing motion, NOT an emergency-stop or zero-speed detector."""
from collections import deque
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class StabilityLimits:
    window_s: float = 1.5
    max_age_s: float = .3
    max_gap_s: float = .2
    min_samples: int = 15
    max_position_span_m: float = .06
    max_yaw_span_rad: float = .15
    max_linear_rms_mps: float = .06
    max_angular_rms_rps: float = .12
    max_linear_peak_mps: float = .20
    max_angular_peak_rps: float = .30

    def __post_init__(self):
        for name, value in vars(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError('Invalid stability limit: ' + name)
        if self.min_samples < 2 or int(self.min_samples) != self.min_samples:
            raise ValueError('min_samples must be an integer >=2')


class BalanceObservationGate:
    def __init__(self, limits=None):
        self.limits = limits or StabilityLimits()
        self.samples = deque(maxlen=2000)
        self.reason = 'WAITING_FOR_ODOMETRY'
        self.frame = None

    def clear(self, reason='WINDOW_RESET'):
        self.samples.clear()
        self.reason = reason

    def add(self, stamp, ros_now, receipt, frame, x, y, yaw, vx, vy, wz):
        values = (stamp, ros_now, receipt, x, y, yaw, vx, vy, wz)
        if (not all(math.isfinite(v) for v in values) or stamp <= 0 or
                not frame or not 0 <= ros_now-stamp <= self.limits.max_age_s):
            self.clear('INVALID_OR_STALE_ODOMETRY')
            return False
        if self.frame is not None and self.frame != frame:
            self.clear('ODOMETRY_FRAME_CHANGED')
        self.frame = frame
        if self.samples:
            previous = self.samples[-1]
            if stamp <= previous[0] or receipt <= previous[1]:
                self.clear('ODOMETRY_TIME_NOT_ADVANCING')
                return False
            if (stamp-previous[0] > self.limits.max_gap_s or
                    receipt-previous[1] > self.limits.max_gap_s):
                self.clear('ODOMETRY_GAP')
        self.samples.append((stamp, receipt, x, y, yaw, math.hypot(vx, vy), abs(wz)))
        # Retain the sample immediately before the window boundary.
        while len(self.samples) > 2 and receipt-self.samples[1][1] >= self.limits.window_s:
            self.samples.popleft()
        return True

    def evaluate(self, ros_now, steady_now):
        l, rows = self.limits, list(self.samples)
        if not rows:
            return False, self.reason, {}
        if not (0 <= ros_now-rows[-1][0] <= l.max_age_s and
                0 <= steady_now-rows[-1][1] <= l.max_age_s):
            return False, 'ODOMETRY_STALE', {}
        if len(rows) < l.min_samples or rows[-1][1]-rows[0][1] < l.window_s:
            return False, 'COLLECTING_STABILITY_WINDOW', {}
        # Span is conservative: diagonal of the xy bounding rectangle.
        span = math.hypot(max(r[2] for r in rows)-min(r[2] for r in rows),
                          max(r[3] for r in rows)-min(r[3] for r in rows))
        # Unwrap increments rather than using min/max of wrapped yaw.
        ys = [0.]
        for a, b in zip(rows, rows[1:]):
            ys.append(ys[-1]+math.atan2(math.sin(b[4]-a[4]), math.cos(b[4]-a[4])))
        metrics = dict(position_span_m=span, yaw_span_rad=max(ys)-min(ys),
                       linear_rms_mps=math.sqrt(sum(r[5]**2 for r in rows)/len(rows)),
                       angular_rms_rps=math.sqrt(sum(r[6]**2 for r in rows)/len(rows)),
                       linear_peak_mps=max(r[5] for r in rows),
                       angular_peak_rps=max(r[6] for r in rows), samples=len(rows))
        for name, limit in [('position_span_m', l.max_position_span_m),
                            ('yaw_span_rad', l.max_yaw_span_rad),
                            ('linear_rms_mps', l.max_linear_rms_mps),
                            ('angular_rms_rps', l.max_angular_rms_rps),
                            ('linear_peak_mps', l.max_linear_peak_mps),
                            ('angular_peak_rps', l.max_angular_peak_rps)]:
            if metrics[name] > limit:
                return False, 'EXCESS_' + name.upper(), metrics
        return True, 'BOUNDED_BALANCING_WINDOW', metrics


@dataclass(frozen=True)
class MotionInterruptLimits:
    """Limits for invalidating localization, not for declaring the robot still."""
    horizon_s: float = .5
    max_age_s: float = .3
    max_gap_s: float = .2
    max_translation_m: float = .15
    max_rotation_rad: float = .35

    def __post_init__(self):
        for name, value in vars(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError('Invalid motion interrupt limit: ' + name)


class MotionInterruptGate:
    """Monitor odometry health and emit events only for substantial motion.

    Small balancing velocity and pose jitter are deliberately ignored.  A
    motion event means that observations already accumulated for localization
    must be discarded; it is not an emergency stop and does not prove that the
    robot is stationary afterwards.
    """
    def __init__(self, limits=None):
        self.limits = limits or MotionInterruptLimits()
        self.samples = deque(maxlen=2000)
        self.reason = 'WAITING_FOR_ODOMETRY'
        self.frame = None
        self.generation = 0

    def clear(self, reason='MONITOR_RESET'):
        self.samples.clear()
        self.reason = reason

    def add(self, stamp, ros_now, receipt, frame, x, y, yaw, vx=0., vy=0., wz=0.):
        del vx, vy, wz  # Velocities do not gate normal balancing operation.
        values = (stamp, ros_now, receipt, x, y, yaw)
        if (not all(math.isfinite(v) for v in values) or stamp <= 0 or
                not frame or not 0 <= ros_now-stamp <= self.limits.max_age_s):
            self.clear('INVALID_OR_STALE_ODOMETRY')
            return False, False, self.reason, {}
        if self.frame is not None and self.frame != frame:
            self.clear('ODOMETRY_FRAME_CHANGED')
            self.frame = frame
            return False, False, self.reason, {}
        self.frame = frame
        if self.samples:
            previous = self.samples[-1]
            if stamp <= previous[0] or receipt <= previous[1]:
                self.clear('ODOMETRY_TIME_NOT_ADVANCING')
                return False, False, self.reason, {}
            if (stamp-previous[0] > self.limits.max_gap_s or
                    receipt-previous[1] > self.limits.max_gap_s):
                self.clear('ODOMETRY_GAP')
                return False, False, self.reason, {}
        self.samples.append((stamp, receipt, x, y, yaw))
        while (len(self.samples) > 2 and
               receipt-self.samples[1][1] >= self.limits.horizon_s):
            self.samples.popleft()
        origin = self.samples[0]
        translation = max(math.hypot(r[2]-origin[2], r[3]-origin[3])
                          for r in self.samples)
        rotation = 0.
        previous_yaw = origin[4]
        unwrapped = 0.
        for row in list(self.samples)[1:]:
            unwrapped += math.atan2(math.sin(row[4]-previous_yaw),
                                    math.cos(row[4]-previous_yaw))
            rotation = max(rotation, abs(unwrapped))
            previous_yaw = row[4]
        metrics = dict(translation_m=translation, rotation_rad=rotation,
                       samples=len(self.samples))
        moved = (translation > self.limits.max_translation_m or
                 rotation > self.limits.max_rotation_rad)
        if moved:
            self.generation += 1
            self.samples.clear()
            self.samples.append((stamp, receipt, x, y, yaw))
            self.reason = 'MOTION_INTERRUPT'
            return True, True, self.reason, metrics
        self.reason = 'MOTION_WITHIN_INTERRUPT_LIMITS'
        return True, False, self.reason, metrics

    def evaluate(self, ros_now, steady_now):
        if not self.samples:
            return False, self.reason, {}
        latest = self.samples[-1]
        if not (0 <= ros_now-latest[0] <= self.limits.max_age_s and
                0 <= steady_now-latest[1] <= self.limits.max_age_s):
            return False, 'ODOMETRY_STALE', {}
        return True, self.reason, dict(samples=len(self.samples),
                                       generation=self.generation)


def navigation_terminal_known(result):
    """A local timeout or cancel acknowledgement is never remote termination."""
    return (not result.timed_out and
            (result.accepted is False or
             (result.accepted is True and result.status in (4, 5, 6))))


def estimate_is_reportable(msg, episode, class_id, class_name, now,
                          not_before, max_age=.5, max_variance=.25):
    """Strict final boundary; quality is not a probability of spatial accuracy."""
    try:
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        c = list(msg.covariance_xy)
        p = msg.position
        if (not msg.valid or msg.header.frame_id != 'map' or
                (msg.episode_id, msg.class_id, msg.class_name) !=
                (episode, class_id, class_name) or
                not stamp >= not_before or not 0 <= now-stamp <= max_age or
                len(c) != 4 or
                not all(math.isfinite(x) for x in [p.x, p.y, p.z, msg.quality]+c)):
            return False
        return (0 < msg.quality <= 1 and 0 <= c[0] <= max_variance and
                0 <= c[3] <= max_variance and abs(c[1]-c[2]) <= 1e-8 and
                c[0]*c[3]-c[1]*c[2] >= -1e-10 and bool(msg.method))
    except (TypeError, AttributeError, ValueError):
        return False
