"""Deployment-equivalent shortlist and 12-feature construction."""

from math import isfinite, pi

import numpy as np

from .model import FEATURE_NAMES


def wrap_angle(angle):
    values = np.asarray(angle, dtype=np.float64)
    return np.arctan2(np.sin(values), np.cos(values))


def action_times(path_distances_m, turn_angles_rad, linear_speed, angular_speed, observation_time):
    if not all(isfinite(v) and v > 0.0 for v in (linear_speed, angular_speed)):
        raise ValueError("linear_speed and angular_speed must be finite and positive")
    if not isfinite(observation_time) or observation_time < 0.0:
        raise ValueError("observation_time must be finite and nonnegative")
    return (
        np.asarray(path_distances_m, dtype=np.float64) / linear_speed
        + np.asarray(turn_angles_rad, dtype=np.float64) / angular_speed
        + observation_time
    )


def make_shortlist(
    gain_counts, path_distances_m, turn_angles_rad,
    linear_speed=0.25, angular_speed=0.8, observation_time=0.25,
    time_discount=20.0, shortlist_size=32, blocked_mask=None,
):
    gains = np.asarray(gain_counts, dtype=np.float64)
    distances = np.asarray(path_distances_m, dtype=np.float64)
    turns = np.asarray(turn_angles_rad, dtype=np.float64)
    if gains.ndim != 1 or distances.shape != gains.shape or turns.shape != gains.shape:
        raise ValueError("gain, path-distance, and turn arrays must be matching 1-D arrays")
    if not isfinite(time_discount) or time_discount <= 0.0:
        raise ValueError("time_discount must be finite and positive")
    if not isinstance(shortlist_size, (int, np.integer)) or shortlist_size < 1:
        raise ValueError("shortlist_size must be positive")
    times = action_times(distances, np.abs(turns), linear_speed, angular_speed, observation_time)
    valid = np.isfinite(distances) & np.isfinite(turns) & (gains > 0)
    if blocked_mask is not None:
        blocked = np.asarray(blocked_mask, dtype=bool)
        if blocked.shape != gains.shape:
            raise ValueError("blocked_mask has the wrong shape")
        valid &= ~blocked
    if not np.any(valid):
        return np.empty(0, dtype=np.int32), times, np.full(gains.shape, -np.inf)
    utility = gains * np.exp(-times / time_discount)
    utility[~valid] = -np.inf
    count = min(int(shortlist_size), int(np.count_nonzero(valid)))
    # Stable full sort makes ties deterministic across platforms.
    indices = np.flatnonzero(valid)
    order = np.lexsort((indices, -utility[indices]))
    return indices[order[:count]].astype(np.int32), times, utility


def build_candidate_features(
    candidate_indices,
    gain_counts,
    unseen_count,
    searched_count,
    path_distances_m,
    turn_angles_rad,
    clearance_m,
    visit_counts,
    visible_areas_m2,
    visible_counts,
    euclidean_distances_m,
    resolution,
    linear_speed=0.25,
    angular_speed=0.8,
    observation_time=0.25,
):
    """Build features in exactly the order used to train Tiny MLP V2."""
    indices = np.asarray(candidate_indices, dtype=np.int32)
    if indices.ndim != 1 or indices.size == 0:
        raise ValueError("candidate_indices must be a non-empty 1-D array")
    arrays = [
        np.asarray(value) for value in (
            gain_counts, path_distances_m, turn_angles_rad, clearance_m,
            visit_counts, visible_areas_m2, visible_counts, euclidean_distances_m,
        )
    ]
    size = arrays[0].size
    if any(array.ndim != 1 or array.size != size for array in arrays):
        raise ValueError("All per-candidate inputs must be matching 1-D arrays")
    if np.any(indices < 0) or np.any(indices >= size):
        raise IndexError("candidate index is outside the input arrays")
    if unseen_count < 0 or searched_count < 0:
        raise ValueError("pixel counts must be nonnegative")
    if not isfinite(resolution) or resolution <= 0.0:
        raise ValueError("resolution must be finite and positive")

    gains = arrays[0].astype(np.float64)
    distances = arrays[1].astype(np.float64)[indices]
    turns = np.abs(arrays[2].astype(np.float64)[indices])
    times = action_times(distances, turns, linear_speed, angular_speed, observation_time)
    gain_areas = gains[indices] * resolution ** 2
    gain_fractions = gains[indices] / max(int(unseen_count), 1)
    observable_count = int(searched_count) + int(unseen_count)
    coverage = int(searched_count) / max(observable_count, 1)
    visible = arrays[6].astype(np.float64)[indices]
    redundancy = (visible - gains[indices]) / np.maximum(visible, 1.0)
    euclidean = arrays[7].astype(np.float64)[indices]
    detour = distances / np.maximum(euclidean, resolution)
    detour[euclidean < 0.5 * resolution] = 1.0

    features = np.column_stack((
        gain_areas,
        gain_fractions,
        distances,
        turns,
        times,
        arrays[3].astype(np.float64)[indices],
        arrays[4].astype(np.float64)[indices],
        np.full(indices.size, coverage),
        arrays[5].astype(np.float64)[indices],
        redundancy,
        euclidean,
        detour,
    )).astype(np.float32)
    if features.shape[1] != len(FEATURE_NAMES) or not np.all(np.isfinite(features)):
        raise ValueError("Constructed features are invalid")
    return features


def turn_angles(current_yaw, candidate_headings):
    if not isfinite(current_yaw):
        raise ValueError("current_yaw must be finite")
    return np.abs(wrap_angle(np.asarray(candidate_headings) - current_yaw))
