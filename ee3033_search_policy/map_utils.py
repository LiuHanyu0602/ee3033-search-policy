"""OccupancyGrid utilities; no ROS imports or navigation side effects."""

from collections import deque
from dataclasses import dataclass
from math import atan2, ceil, cos, floor, hypot, isfinite, sin, sqrt
from numbers import Integral

import numpy as np


@dataclass(frozen=True)
class GridGeometry:
    """Planar map geometry; row zero is the OccupancyGrid origin side."""

    width: int
    height: int
    resolution: float
    origin_x: float = 0.0
    origin_y: float = 0.0
    origin_yaw: float = 0.0

    def __post_init__(self):
        if not all(
            isinstance(value, Integral) and value > 0
            for value in (self.width, self.height)
        ):
            raise ValueError("Map width and height must be positive integers")
        if not isfinite(self.resolution) or self.resolution <= 0:
            raise ValueError("Map resolution must be finite and positive")
        if not all(isfinite(v) for v in (
            self.origin_x, self.origin_y, self.origin_yaw,
        )):
            raise ValueError("Map origin must be finite")

    @classmethod
    def from_message(cls, message):
        """Read a planar OccupancyGrid, rejecting invalid/tilted orientations."""
        info = message.info
        p, q = info.origin.position, info.origin.orientation
        if not all(isfinite(v) for v in (p.z, q.x, q.y, q.z, q.w)):
            raise ValueError("Map origin contains non-finite values")
        norm = hypot(q.x, q.y, q.z, q.w)
        if abs(norm - 1.0) > 1e-3:
            raise ValueError("Map origin quaternion must be normalized")
        if abs(q.x) > 1e-6 or abs(q.y) > 1e-6:
            raise ValueError("Only planar map origins are supported")
        return cls(
            info.width, info.height, info.resolution, p.x, p.y,
            atan2(2 * q.w * q.z, 1 - 2 * q.z ** 2),
        )


def occupancy_array(message):
    """Copy ROS row-major [-1, 100] data without the saved-image vertical flip."""
    geometry = GridGeometry.from_message(message)
    values = np.asarray(message.data)
    if values.size != geometry.width * geometry.height:
        raise ValueError("OccupancyGrid data length does not match dimensions")
    if not np.issubdtype(values.dtype, np.integer):
        raise ValueError("OccupancyGrid data must contain integers")
    if np.any((values < -1) | (values > 100)):
        raise ValueError("OccupancyGrid values must be in [-1, 100]")
    return values.astype(np.int8).reshape(
        geometry.height, geometry.width,
    )


def grid_to_world(row, col, geometry):
    """Convert a valid (row, column) to its world-space cell centre."""
    if not isinstance(row, Integral) or not isinstance(col, Integral):
        raise ValueError("Grid indices must be integers")
    if not (0 <= row < geometry.height and 0 <= col < geometry.width):
        raise ValueError("Grid index is outside the map")
    x = (col + 0.5) * geometry.resolution
    y = (row + 0.5) * geometry.resolution
    c, s = cos(geometry.origin_yaw), sin(geometry.origin_yaw)
    return (
        float(geometry.origin_x + c * x - s * y),
        float(geometry.origin_y + s * x + c * y),
    )


def world_to_grid(x, y, geometry):
    """Apply inverse origin rotation and floor; reject out-of-map coordinates."""
    if not isfinite(x) or not isfinite(y):
        raise ValueError("World coordinates must be finite")
    dx, dy = x - geometry.origin_x, y - geometry.origin_y
    c, s = cos(geometry.origin_yaw), sin(geometry.origin_yaw)
    col = floor((c * dx + s * dy) / geometry.resolution)
    row = floor((-s * dx + c * dy) / geometry.resolution)
    if not (0 <= row < geometry.height and 0 <= col < geometry.width):
        raise ValueError("World coordinate is outside the map")
    return row, col


def classify_cells(values, free_threshold=20, occupied_threshold=65):
    """Return free, occupied, and unknown/ambiguous masks; -1 is never free."""
    if not 0 <= free_threshold < occupied_threshold <= 100:
        raise ValueError("Require 0 <= free_threshold < occupied_threshold <= 100")
    values = np.asarray(values)
    if values.ndim != 2 or not np.issubdtype(values.dtype, np.integer):
        raise ValueError("Occupancy values must be a 2-D integer array")
    if np.any((values < -1) | (values > 100)):
        raise ValueError("Occupancy values must be in [-1, 100]")
    free = (values >= 0) & (values <= free_threshold)
    occupied = values >= occupied_threshold
    return free, occupied, ~(free | occupied)


def safe_candidate_mask(
    values, resolution, clearance_m=0.20,
    free_threshold=20, occupied_threshold=65,
):
    """Conservatively clear cell centres from blocked cell squares and edges.

    A disk plus half a cell diagonal bounds distance to each blocked square.
    This deliberately over-inflates some cells rather than understating clearance.
    NumPy slices avoid per-cell Python work or a SciPy/OpenCV dependency.
    """
    if not isfinite(resolution) or resolution <= 0:
        raise ValueError("resolution must be finite and positive")
    if not isfinite(clearance_m) or clearance_m < 0:
        raise ValueError("clearance_m must be finite and nonnegative")
    free, _, _ = classify_cells(values, free_threshold, occupied_threshold)
    height, width = free.shape
    safe = free.copy()
    if height == 0 or width == 0:
        raise ValueError("Map must not be empty")
    rows, cols = np.indices(free.shape)
    edge_clearance = np.minimum(
        np.minimum(rows + 0.5, height - rows - 0.5),
        np.minimum(cols + 0.5, width - cols - 0.5),
    ) * resolution
    safe &= edge_clearance >= clearance_m
    if not np.any(safe):
        return safe
    radius = clearance_m / resolution + sqrt(0.5)
    reach = ceil(radius)
    blocked = ~free
    for dr in range(-min(reach, height - 1), min(reach, height - 1) + 1):
        for dc in range(-min(reach, width - 1), min(reach, width - 1) + 1):
            if dr * dr + dc * dc >= radius * radius:
                continue
            r0, r1 = max(0, -dr), min(height, height - dr)
            c0, c1 = max(0, -dc), min(width, width - dc)
            safe[r0:r1, c0:c1] &= ~blocked[
                r0 + dr:r1 + dr, c0 + dc:c1 + dc,
            ]
    return safe


def select_test_positions(safe_mask, geometry, min_separation_m=1.0):
    """Select a deterministic separated pair in the largest safe component.

    Four-connected flood fill avoids diagonal corner cutting. Two Euclidean
    farthest-point sweeps give separated endpoints, not an exact diameter.
    Returns two world-coordinate (x, y) pairs.
    """
    safe = np.asarray(safe_mask, dtype=bool)
    if safe.shape != (geometry.height, geometry.width):
        raise ValueError("Mask shape does not match map geometry")
    if not isfinite(min_separation_m) or min_separation_m <= 0:
        raise ValueError("min_separation_m must be finite and positive")
    visited = np.zeros_like(safe)

    def flood(start, seen):
        queue = deque([start])
        seen[start] = True
        component = []
        while queue:
            row, col = queue.popleft()
            component.append((row, col))
            for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                nr, nc = row + dr, col + dc
                if (
                    0 <= nr < geometry.height and 0 <= nc < geometry.width
                    and safe[nr, nc] and not seen[nr, nc]
                ):
                    seen[nr, nc] = True
                    queue.append((nr, nc))
        return component

    largest = []
    for row, col in np.argwhere(safe):
        start = (int(row), int(col))
        if not visited[start]:
            component = flood(start, visited)
            if len(component) > len(largest):
                largest = component
    if len(largest) < 2:
        raise ValueError("Map has no safe connected component with two cells")
    cells = np.asarray(largest)
    first = cells[np.argmax(np.sum((cells - cells[0]) ** 2, axis=1))]
    second = cells[np.argmax(np.sum((cells - first) ** 2, axis=1))]
    distance = sqrt(
        (first[0] - second[0]) ** 2 + (first[1] - second[1]) ** 2,
    ) * geometry.resolution
    if distance < min_separation_m:
        raise ValueError(
            f"Selected pair separation {distance:.3f} m is below "
            f"{min_separation_m:.3f} m; reduce min_separation_m or clearance",
        )
    return grid_to_world(*first, geometry), grid_to_world(*second, geometry)
