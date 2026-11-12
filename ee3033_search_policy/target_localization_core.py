"""Hardware-independent geometry for robust 2-D target localization.

The functions in this module deliberately have no ROS dependency so they can
be unit-tested on a development computer and reused by a ROS 2 adapter.
"""

from dataclasses import dataclass
import math

import numpy as np


EPS = 1.0e-9


def wrap_angle(angle):
    """Wrap radians to [-pi, pi)."""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class CameraModel:
    fx: float
    fy: float
    cx: float
    cy: float
    distortion: tuple = ()

    def __post_init__(self):
        if min(self.fx, self.fy) <= 0.0:
            raise ValueError("camera focal lengths must be positive")


def undistort_normalized(xd, yd, distortion=(), iterations=8):
    """Invert ROS plumb_bob distortion using fixed-point iteration.

    Coefficients follow sensor_msgs/CameraInfo: k1,k2,t1,t2,k3[,k4,k5,k6].
    Rational denominator terms are supported when present.
    """
    d = list(distortion) + [0.0] * (8 - len(distortion))
    k1, k2, p1, p2, k3, k4, k5, k6 = d[:8]
    x, y = float(xd), float(yd)
    for _ in range(max(1, int(iterations))):
        r2 = x * x + y * y
        r4, r6 = r2 * r2, r2 * r2 * r2
        numerator = 1.0 + k1 * r2 + k2 * r4 + k3 * r6
        denominator = 1.0 + k4 * r2 + k5 * r4 + k6 * r6
        radial = numerator / max(abs(denominator), EPS)
        dx = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
        dy = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
        if abs(radial) < EPS:
            break
        x = (xd - dx) / radial
        y = (yd - dy) / radial
    return np.array([x, y], dtype=np.float64)


def pixel_to_camera_ray(u, v, camera):
    """Return a unit ray in optical coordinates (+x right,+y down,+z forward)."""
    xd = (float(u) - camera.cx) / camera.fx
    yd = (float(v) - camera.cy) / camera.fy
    x, y = undistort_normalized(xd, yd, camera.distortion)
    ray = np.array([x, y, 1.0], dtype=np.float64)
    return ray / np.linalg.norm(ray)


def bbox_contact_pixel(center_x, center_y, width, height, image_height=None):
    """Bottom-centre pixel, used as the target/ground contact point."""
    if width <= 0 or height <= 0:
        raise ValueError("bounding-box dimensions must be positive")
    u = float(center_x)
    v = float(center_y) + 0.5 * float(height)
    if image_height is not None:
        v = min(max(v, 0.0), float(image_height - 1))
    return u, v


def ray_ground_intersection(ray_camera, rotation_base_from_camera,
                            camera_origin_base, ground_z=0.0,
                            min_distance=0.05, max_distance=10.0):
    """Intersect an optical ray with the base-frame horizontal ground plane."""
    ray = np.asarray(rotation_base_from_camera, dtype=np.float64) @ np.asarray(
        ray_camera, dtype=np.float64
    )
    origin = np.asarray(camera_origin_base, dtype=np.float64)
    if ray.shape != (3,) or origin.shape != (3,):
        raise ValueError("ray and camera origin must be 3-vectors")
    if abs(ray[2]) < EPS:
        return None
    scale = (float(ground_z) - origin[2]) / ray[2]
    if scale <= 0.0:
        return None
    point = origin + scale * ray
    planar = float(np.linalg.norm(point[:2] - origin[:2]))
    if not min_distance <= planar <= max_distance:
        return None
    return point


def robust_lidar_range(ranges, angles, target_bearing, window_rad,
                       min_range, max_range, mad_scale=3.5,
                       minimum_points=2):
    """Median LiDAR association with circular-angle and MAD outlier gating."""
    ranges = np.asarray(ranges, dtype=np.float64)
    angles = np.asarray(angles, dtype=np.float64)
    delta = np.arctan2(
        np.sin(angles - float(target_bearing)),
        np.cos(angles - float(target_bearing)),
    )
    valid = (
        np.isfinite(ranges)
        & (ranges >= float(min_range))
        & (ranges <= float(max_range))
        & (np.abs(delta) <= float(window_rad))
    )
    values = ranges[valid]
    if values.size < int(minimum_points):
        return None
    median = float(np.median(values))
    mad = float(np.median(np.abs(values - median)))
    if mad > EPS:
        values = values[np.abs(values - median) <= mad_scale * 1.4826 * mad]
    if values.size < int(minimum_points):
        return None
    return float(np.median(values))


def nearest_supported_lidar_cluster(
        ranges, angles, target_bearing, edge_bearings, angular_margin_rad,
        min_range, max_range, minimum_points=2):
    """Select the nearest adjacent-beam support inside the bbox angle sector.

    There is deliberately no required target-to-wall depth separation. This
    selector itself does not apply the later camera-distance agreement gate.
    A lone short return cannot be selected by itself because
    ``minimum_points`` consecutive valid scan samples are required.
    """
    ranges = np.asarray(ranges, dtype=np.float64)
    angles = np.asarray(angles, dtype=np.float64)
    if ranges.shape != angles.shape or ranges.ndim != 1:
        raise ValueError('LiDAR ranges and angles must be equal-length vectors')
    count = int(minimum_points)
    margin = float(angular_margin_rad)
    if count < 1 or margin < 0 or not math.isfinite(margin):
        raise ValueError('Invalid nearest-cluster configuration')
    centre = float(target_bearing)
    edges = np.asarray(edge_bearings, dtype=np.float64)
    if edges.shape != (2,) or not np.all(np.isfinite(edges)):
        raise ValueError('Two finite bbox edge bearings required')
    edge_delta = np.arctan2(np.sin(edges-centre), np.cos(edges-centre))
    lower, upper = float(np.min(edge_delta)-margin), float(np.max(edge_delta)+margin)
    delta = np.arctan2(np.sin(angles-centre), np.cos(angles-centre))
    valid = (np.isfinite(ranges) & np.isfinite(angles) &
             (ranges >= float(min_range)) & (ranges <= float(max_range)) &
             (delta >= lower) & (delta <= upper))
    indices = np.flatnonzero(valid)
    best = None
    for offset in range(max(0, len(indices)-count+1)):
        window = indices[offset:offset+count]
        if len(window) != count or np.any(np.diff(window) != 1):
            continue
        distance = float(np.median(ranges[window]))
        angle = math.atan2(
            float(np.mean(np.sin(angles[window]))),
            float(np.mean(np.cos(angles[window]))),
        )
        score = (distance, abs(wrap_angle(angle-centre)))
        if best is None or score < best[0]:
            best = (score, distance, angle)
    return None if best is None else (best[1], best[2])


def choose_gated_range(ground_range, lidar_range, gate_m=0.35,
                       prefer_lidar=True):
    """Reject a likely wall return when LiDAR disagrees with ground geometry."""
    ground_ok = ground_range is not None and np.isfinite(ground_range)
    lidar_ok = lidar_range is not None and np.isfinite(lidar_range)
    if ground_ok and lidar_ok and abs(lidar_range - ground_range) <= gate_m:
        return (float(lidar_range), "lidar_ground_consistent") if prefer_lidar else (
            float(ground_range), "ground_lidar_consistent"
        )
    if ground_ok:
        return float(ground_range), (
            "ground_lidar_rejected" if lidar_ok else "ground_plane"
        )
    if lidar_ok:
        return float(lidar_range), "lidar_ungated"
    return None, "unavailable"


def bearing_direction(robot_yaw, camera_yaw_offset, image_bearing):
    angle = wrap_angle(robot_yaw + camera_yaw_offset + image_bearing)
    return np.array([math.cos(angle), math.sin(angle)], dtype=np.float64)


def triangulate_bearings(origins, directions, weights=None):
    """Weighted least-squares intersection of 2-D bearing lines."""
    origins = np.asarray(origins, dtype=np.float64)
    directions = np.asarray(directions, dtype=np.float64)
    if origins.ndim != 2 or origins.shape[1] != 2 or origins.shape != directions.shape:
        raise ValueError("origins and directions must both be Nx2")
    if len(origins) < 2:
        return None
    norms = np.linalg.norm(directions, axis=1)
    if np.any(norms < EPS):
        return None
    directions = directions / norms[:, None]
    weights = np.ones(len(origins)) if weights is None else np.asarray(weights, dtype=np.float64)
    if weights.shape != (len(origins),) or np.any(weights < 0.0):
        raise ValueError("weights must be a non-negative N-vector")
    a = np.zeros((2, 2), dtype=np.float64)
    b = np.zeros(2, dtype=np.float64)
    identity = np.eye(2)
    for origin, direction, weight in zip(origins, directions, weights):
        projector = identity - np.outer(direction, direction)
        a += weight * projector
        b += weight * projector @ origin
    if np.linalg.cond(a) > 1.0e8:
        return None
    return np.linalg.solve(a, b)


def perpendicular_residuals(point, origins, directions):
    point = np.asarray(point, dtype=np.float64)
    origins = np.asarray(origins, dtype=np.float64)
    directions = np.asarray(directions, dtype=np.float64)
    directions = directions / np.linalg.norm(directions, axis=1)[:, None]
    delta = point[None, :] - origins
    return np.abs(delta[:, 0] * directions[:, 1] - delta[:, 1] * directions[:, 0])


def ransac_triangulate(origins, directions, weights=None, threshold_m=0.20,
                       iterations=100, seed=3033):
    """Robust bearing-only triangulation. Returns point and inlier mask."""
    origins = np.asarray(origins, dtype=np.float64)
    directions = np.asarray(directions, dtype=np.float64)
    count = len(origins)
    if count < 2:
        return None, np.zeros(count, dtype=bool)
    weights = np.ones(count) if weights is None else np.asarray(weights, dtype=np.float64)
    rng = np.random.default_rng(seed)
    best = None
    best_score = (-1, -np.inf)
    for _ in range(max(1, int(iterations))):
        pair = rng.choice(count, size=2, replace=False)
        candidate = triangulate_bearings(origins[pair], directions[pair], weights[pair])
        if candidate is None:
            continue
        residuals = perpendicular_residuals(candidate, origins, directions)
        inliers = residuals <= threshold_m
        score = (int(np.count_nonzero(inliers)), -float(np.sum(residuals[inliers])))
        if score > best_score:
            best_score, best = score, inliers
    if best is None or np.count_nonzero(best) < 2:
        return None, np.zeros(count, dtype=bool)
    point = triangulate_bearings(origins[best], directions[best], weights[best])
    return point, best


def robust_weighted_fusion(points, weights=None, gate_m=0.35):
    """Median-gate point estimates, then return weighted mean and covariance."""
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) == 0:
        return None, None, np.zeros(len(points), dtype=bool)
    weights = np.ones(len(points)) if weights is None else np.asarray(weights, dtype=np.float64)
    centre = np.median(points, axis=0)
    inliers = np.linalg.norm(points - centre, axis=1) <= float(gate_m)
    if not np.any(inliers):
        return None, None, inliers
    kept, kept_weights = points[inliers], np.maximum(weights[inliers], EPS)
    mean = np.average(kept, axis=0, weights=kept_weights)
    residual = kept - mean
    covariance = (
        (residual * kept_weights[:, None]).T @ residual / np.sum(kept_weights)
        if len(kept) > 1 else np.eye(2) * gate_m ** 2
    )
    return mean, covariance, inliers
