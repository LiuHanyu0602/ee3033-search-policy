"""Runtime candidate-viewpoint generation from OccupancyGrid-style arrays.

The module has no ROS imports.  A ROS node can pass the array returned by
``map_utils.occupancy_array`` together with ``GridGeometry``.
"""

from collections import deque
from dataclasses import dataclass
import heapq
from math import cos, hypot, isfinite, pi, sin, sqrt

import numpy as np

from .map_utils import GridGeometry, classify_cells, grid_to_world


@dataclass(frozen=True)
class CandidateViewpoints:
    rows: np.ndarray
    cols: np.ndarray
    world_x: np.ndarray
    world_y: np.ndarray
    headings: np.ndarray
    visible_areas_m2: np.ndarray
    visibility_masks: np.ndarray
    clearance_m: np.ndarray
    observable_mask: np.ndarray
    safe_mask: np.ndarray
    clearance_grid_m: np.ndarray

    def __len__(self):
        return int(self.rows.size)


def wrap_angle(angle):
    return (np.asarray(angle) + pi) % (2.0 * pi) - pi


def clearance_map(free_mask, resolution):
    """Conservative 8-connected distance from free cell centres to blockage.

    Unknown cells, occupied cells, and the outside of the map are obstacles.
    The half-cell diagonal converts centre distance into a conservative
    clearance to the obstacle-cell square.
    """
    free = np.asarray(free_mask, dtype=bool)
    if free.ndim != 2 or not free.size:
        raise ValueError("free_mask must be a non-empty 2-D array")
    if not isfinite(resolution) or resolution <= 0.0:
        raise ValueError("resolution must be finite and positive")

    height, width = free.shape
    distance = np.full((height, width), np.inf, dtype=np.float64)
    queue = []
    blocked_rows, blocked_cols = np.nonzero(~free)
    for row, col in zip(blocked_rows, blocked_cols):
        distance[row, col] = 0.0
        heapq.heappush(queue, (0.0, int(row), int(col)))

    # Unit-cost 8-neighbour propagation is the Chebyshev distance.  It is a
    # lower bound on Euclidean clearance, which is intentionally conservative
    # for candidate safety.  Using sqrt(2) on diagonals would be a closer
    # approximation but can overestimate Euclidean distance in other slopes.
    moves = (
        (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
        (-1, -1, 1.0), (-1, 1, 1.0),
        (1, -1, 1.0), (1, 1, 1.0),
    )
    while queue:
        value, row, col = heapq.heappop(queue)
        if value > distance[row, col] + 1e-12:
            continue
        for dr, dc, cost in moves:
            nr, nc = row + dr, col + dc
            if 0 <= nr < height and 0 <= nc < width:
                candidate = value + cost
                if candidate + 1e-12 < distance[nr, nc]:
                    distance[nr, nc] = candidate
                    heapq.heappush(queue, (candidate, nr, nc))

    # Distance to the map boundary is also a hard constraint.
    rows, cols = np.indices(free.shape)
    edge = np.minimum.reduce((
        rows + 0.5, height - rows - 0.5,
        cols + 0.5, width - cols - 0.5,
    ))
    obstacle = np.maximum(distance - sqrt(0.5), 0.0)
    result = np.minimum(obstacle, edge) * resolution
    result[~free] = 0.0
    return result.astype(np.float32)


def reachable_component(safe_mask, start):
    """Return the four-connected safe component containing ``start``.

    Four-connectivity deliberately prevents diagonal corner cutting.  An
    out-of-map or unsafe start returns an empty component so callers can fail
    closed before submitting a planning request.
    """
    safe = np.asarray(safe_mask, dtype=bool)
    if safe.ndim != 2 or not safe.size:
        raise ValueError("safe_mask must be a non-empty 2-D array")
    if not isinstance(start, tuple) or len(start) != 2:
        raise ValueError("start must be a (row, column) tuple")
    row, col = start
    if not isinstance(row, (int, np.integer)) or not isinstance(col, (int, np.integer)):
        raise ValueError("start indices must be integers")
    reachable = np.zeros_like(safe)
    if not (0 <= row < safe.shape[0] and 0 <= col < safe.shape[1]) or not safe[row, col]:
        return reachable
    queue = deque([(int(row), int(col))])
    reachable[row, col] = True
    while queue:
        row, col = queue.popleft()
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            nr, nc = row + dr, col + dc
            if (0 <= nr < safe.shape[0] and 0 <= nc < safe.shape[1] and
                    safe[nr, nc] and not reachable[nr, nc]):
                reachable[nr, nc] = True
                queue.append((nr, nc))
    return reachable


def generate_candidate_positions(safe_mask, clearance_m, spacing_px):
    safe = np.asarray(safe_mask, dtype=bool)
    clearance = np.asarray(clearance_m, dtype=np.float32)
    if safe.ndim != 2 or safe.shape != clearance.shape:
        raise ValueError("safe_mask and clearance_m must be matching 2-D arrays")
    if not isinstance(spacing_px, (int, np.integer)) or spacing_px < 1:
        raise ValueError("spacing_px must be a positive integer")

    height, width = safe.shape
    positions = []
    for row_start in range(0, height, spacing_px):
        for col_start in range(0, width, spacing_px):
            row_end = min(row_start + spacing_px, height)
            col_end = min(col_start + spacing_px, width)
            block = safe[row_start:row_end, col_start:col_end]
            pixels = np.argwhere(block)
            if pixels.size == 0:
                continue
            values = clearance[row_start:row_end, col_start:col_end][block]
            local_row, local_col = pixels[int(np.argmax(values))]
            positions.append((row_start + int(local_row), col_start + int(local_col)))
    return positions


def has_line_of_sight(free_mask, start_row, start_col, end_row, end_col):
    free = np.asarray(free_mask, dtype=bool)
    height, width = free.shape
    x0, y0 = int(start_col), int(start_row)
    x1, y1 = int(end_col), int(end_row)
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    step_x = 1 if x0 < x1 else -1
    step_y = 1 if y0 < y1 else -1
    error = dx - dy
    while True:
        if not (0 <= y0 < height and 0 <= x0 < width and free[y0, x0]):
            return False
        if x0 == x1 and y0 == y1:
            return True
        double_error = 2 * error
        if double_error > -dy:
            error -= dy
            x0 += step_x
        if double_error < dx:
            error += dx
            y0 += step_y


def raycast_visibility(
    free_mask, start_row, start_col, local_heading, field_of_view, max_range_px,
):
    free = np.asarray(free_mask, dtype=bool)
    if not (0.0 < field_of_view <= 2.0 * pi):
        raise ValueError("field_of_view must be in (0, 2*pi]")
    if not isfinite(max_range_px) or max_range_px <= 0.0:
        raise ValueError("max_range_px must be finite and positive")
    if not free[int(start_row), int(start_col)]:
        raise ValueError("Viewpoint must lie in free space")

    visible = np.zeros_like(free)
    free_rows, free_cols = np.nonzero(free)
    delta_x = free_cols - int(start_col)
    # OccupancyGrid row increases along local +y (unlike a saved PGM image).
    delta_y = free_rows - int(start_row)
    distance_squared = delta_x.astype(float) ** 2 + delta_y.astype(float) ** 2
    bearings = np.arctan2(delta_y, delta_x)
    errors = np.arctan2(
        np.sin(bearings - local_heading), np.cos(bearings - local_heading),
    )
    possible = (
        (distance_squared <= max_range_px ** 2)
        & (np.abs(errors) <= field_of_view / 2.0)
    )
    for row, col in zip(free_rows[possible], free_cols[possible]):
        if has_line_of_sight(free, start_row, start_col, row, col):
            visible[row, col] = True
    visible[int(start_row), int(start_col)] = True
    return visible


def generate_viewpoints_from_grid(
    values,
    geometry,
    spacing_m=0.75,
    clearance_m=0.18,
    camera_fov_deg=70.0,
    camera_range_m=3.0,
    headings_per_position=4,
    free_threshold=20,
    occupied_threshold=65,
):
    """Generate safe oriented viewpoints directly from an occupancy grid."""
    if not isinstance(geometry, GridGeometry):
        raise TypeError("geometry must be GridGeometry")
    grid = np.asarray(values)
    if grid.shape != (geometry.height, geometry.width):
        raise ValueError("values shape does not match geometry")
    if not isfinite(spacing_m) or spacing_m <= 0.0:
        raise ValueError("spacing_m must be finite and positive")
    if not isfinite(clearance_m) or clearance_m < 0.0:
        raise ValueError("clearance_m must be finite and nonnegative")
    if not isfinite(camera_fov_deg) or not 0.0 < camera_fov_deg <= 360.0:
        raise ValueError("camera_fov_deg must be in (0, 360]")
    if not isfinite(camera_range_m) or camera_range_m <= 0.0:
        raise ValueError("camera_range_m must be finite and positive")
    if not isinstance(headings_per_position, (int, np.integer)) or headings_per_position < 1:
        raise ValueError("headings_per_position must be a positive integer")

    free, _, _ = classify_cells(grid, free_threshold, occupied_threshold)
    clearance = clearance_map(free, geometry.resolution)
    safe = free & (clearance >= clearance_m)
    spacing_px = max(1, int(round(spacing_m / geometry.resolution)))
    positions = generate_candidate_positions(safe, clearance, spacing_px)
    if not positions:
        raise ValueError("No safe candidate positions were generated")

    local_headings = np.arange(headings_per_position, dtype=float) * (
        2.0 * pi / headings_per_position
    )
    rows, cols, xs, ys, headings, areas, masks, candidate_clearance = ([] for _ in range(8))
    for row, col in positions:
        world_x, world_y = grid_to_world(row, col, geometry)
        for local_heading in local_headings:
            mask = raycast_visibility(
                free, row, col, float(local_heading),
                np.deg2rad(camera_fov_deg), camera_range_m / geometry.resolution,
            )
            rows.append(row)
            cols.append(col)
            xs.append(world_x)
            ys.append(world_y)
            headings.append(float(wrap_angle(local_heading + geometry.origin_yaw)))
            areas.append(np.count_nonzero(mask) * geometry.resolution ** 2)
            masks.append(mask)
            candidate_clearance.append(clearance[row, col])

    visibility = np.stack(masks).astype(bool)
    return CandidateViewpoints(
        rows=np.asarray(rows, dtype=np.int32),
        cols=np.asarray(cols, dtype=np.int32),
        world_x=np.asarray(xs, dtype=np.float32),
        world_y=np.asarray(ys, dtype=np.float32),
        headings=np.asarray(headings, dtype=np.float32),
        visible_areas_m2=np.asarray(areas, dtype=np.float32),
        visibility_masks=visibility,
        clearance_m=np.asarray(candidate_clearance, dtype=np.float32),
        observable_mask=np.any(visibility, axis=0) & free,
        safe_mask=safe,
        clearance_grid_m=clearance,
    )
