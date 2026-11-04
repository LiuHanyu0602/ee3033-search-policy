"""Validated map-frame ray geometry and episode-scoped observation memory."""
from collections import deque
import math
import numpy as np
from .target_localization_core import ransac_triangulate, robust_weighted_fusion


def rotation_from_quaternion(x, y, z, w):
    q = np.asarray([x, y, z, w], dtype=float)
    n = np.linalg.norm(q)
    if not np.all(np.isfinite(q)) or n < 1e-9:
        raise ValueError('Invalid transform quaternion')
    x, y, z, w = q / n
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def map_ground_point(origin, direction, ground_z, minimum_range=.15,
                     maximum_range=4., minimum_down_angle=.0873):
    o, d = np.asarray(origin, dtype=float), np.asarray(direction, dtype=float)
    if o.shape != (3,) or d.shape != (3,) or not np.all(np.isfinite([o, d])):
        raise ValueError('Finite map-frame origin and ray required')
    norm = np.linalg.norm(d)
    if norm < 1e-9 or not math.isfinite(ground_z):
        raise ValueError('Invalid ray or ground plane')
    d = d/norm
    if o[2] <= ground_z or d[2] >= -math.sin(minimum_down_angle):
        return None
    point = o + ((ground_z-o[2])/d[2])*d
    distance = np.linalg.norm(point[:2]-o[:2])
    return point[:2] if minimum_range <= distance <= maximum_range else None


class LocalizationMemory:
    def __init__(self, maximum=30, baseline=.25, crossing_angle=.17453,
                 fusion_gate=.35, range_min=.15, range_max=4., horizon=15.):
        self.maximum, self.baseline, self.crossing_angle = maximum, baseline, crossing_angle
        self.fusion_gate, self.range_min, self.range_max = fusion_gate, range_min, range_max
        self.horizon = horizon
        self.configure('', -1, '')

    def configure(self, episode, class_id, name):
        self.episode, self.class_id, self.name = episode, class_id, name
        self.last_stamp = None
        self.samples = deque(maxlen=self.maximum)

    def accept(self, observation, now, age_limit=.5):
        stamp = observation.header.stamp.sec+observation.header.stamp.nanosec*1e-9
        if (not self.episode or observation.episode_id != self.episode or
                observation.class_id != self.class_id or observation.class_name != self.name or
                not math.isfinite(now) or stamp <= 0 or not 0 <= now-stamp <= age_limit or
                (self.last_stamp is not None and stamp <= self.last_stamp)):
            return False
        self.last_stamp = stamp
        return True

    def add(self, stamp, origin, direction, ground, weight, track=''):
        origin, direction = np.asarray(origin, dtype=float), np.asarray(direction, dtype=float)
        if not np.all(np.isfinite(origin)) or not np.all(np.isfinite(direction)):
            raise ValueError('Nonfinite observation geometry')
        planar = np.linalg.norm(direction[:2])
        if planar < 1e-9 or not math.isfinite(weight) or not 0 < weight <= 1:
            raise ValueError('Invalid ray or confidence')
        if self.samples and track and self.samples[-1][5] and track != self.samples[-1][5]:
            self.samples.clear()
        while self.samples and stamp-self.samples[0][0] > self.horizon:
            self.samples.popleft()
        # Repeated views can improve range fusion, but only distinct origins are
        # admitted to bearing triangulation below.
        self.samples.append((stamp, origin[:2], direction[:2]/planar, ground, weight, track))

    def estimate(self):
        points, weights = [], []
        rays = []
        for sample in self.samples:
            _, origin, direction, ground, weight, _ = sample
            if ground is not None:
                points.append(ground); weights.append(weight)
            if all(np.linalg.norm(origin-r[1]) >= .03 for r in rays):
                rays.append(sample)
        method = 'ground_projection_fusion'
        if len(rays) >= 3:
            origins = np.asarray([r[1] for r in rays])
            directions = np.asarray([r[2] for r in rays])
            baseline = np.max(np.linalg.norm(origins[:, None]-origins[None, :], axis=2))
            cross = np.max(np.abs(directions[:, None, 0]*directions[None, :, 1]-
                                  directions[:, None, 1]*directions[None, :, 0]))
            if baseline >= self.baseline and cross >= math.sin(self.crossing_angle):
                p, mask = ransac_triangulate(origins, directions,
                    [r[4] for r in rays], threshold_m=.20)
                if p is not None and np.count_nonzero(mask) >= 3:
                    delta = p-origins[mask]
                    forward = np.sum(delta*directions[mask], axis=1)
                    distances = np.linalg.norm(delta, axis=1)
                    if np.all(forward > 0) and np.all((distances >= self.range_min) & (distances <= self.range_max)):
                        # Ground and bearing disagreement is a failure, not blind averaging.
                        if points and np.linalg.norm(p-np.median(points, axis=0)) > self.fusion_gate:
                            return None, None, 'INCONSISTENT_METHODS'
                        points.append(p); weights.append(float(np.mean([r[4] for r in rays])))
                        method = 'multiview_ransac_fusion'
        if not points:
            return None, None, 'INSUFFICIENT_GEOMETRY'
        p, cov, _ = robust_weighted_fusion(points, weights, self.fusion_gate)
        if p is None or not np.all(np.isfinite(p)) or not np.all(np.isfinite(cov)):
            return None, None, 'INVALID_ESTIMATE'
        # Empirical scatter is NOT total localization uncertainty. Provisional
        # floor prevents repeated pixels implying impossible zero uncertainty.
        return p, cov+np.eye(2)*.01, method
