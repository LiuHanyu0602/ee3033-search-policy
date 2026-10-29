"""Map tests run without ROS or a sourced workspace."""

from math import cos, pi, sin, sqrt
from types import SimpleNamespace as NS

import numpy as np
import pytest

from ee3033_search_policy.map_utils import (
    GridGeometry,
    classify_cells,
    grid_to_world,
    occupancy_array,
    safe_candidate_mask,
    select_test_positions,
    world_to_grid,
)


def map_message(data, width=3, height=2, yaw=0.0):
    return NS(
        data=data,
        info=NS(
            width=width, height=height, resolution=0.5,
            origin=NS(
                position=NS(x=-2.0, y=3.0, z=0.0),
                orientation=NS(x=0.0, y=0.0, z=sin(yaw / 2), w=cos(yaw / 2)),
            ),
        ),
    )


def test_occupancy_array_keeps_ros_row_order_and_copies():
    message = map_message([0, -1, 100, 20, 40, 65])
    values = occupancy_array(message)
    assert values.dtype == np.int8
    np.testing.assert_array_equal(values, [[0, -1, 100], [20, 40, 65]])
    values[0, 0] = 100
    assert message.data[0] == 0


@pytest.mark.parametrize("data", [
    [0], [0, 0, 0, 0, 0, 101], [-2, 0, 0, 0, 0, 0],
    [0.0] * 6,
])
def test_bad_map_data(data):
    with pytest.raises(ValueError):
        occupancy_array(map_message(data))


@pytest.mark.parametrize("yaw", [0, pi / 2, -pi / 3, pi])
def test_rotated_grid_roundtrip(yaw):
    geometry = GridGeometry.from_message(map_message([0] * 6, yaw=yaw))
    for row in range(geometry.height):
        for col in range(geometry.width):
            xy = grid_to_world(row, col, geometry)
            assert world_to_grid(*xy, geometry) == (row, col)


def test_cell_centres_and_no_saved_image_flip():
    geometry = GridGeometry(3, 2, 0.5, -2, 3)
    assert grid_to_world(0, 0, geometry) == (-1.75, 3.25)
    assert grid_to_world(1, 2, geometry) == (-0.75, 3.75)
    rotated = GridGeometry(3, 2, 0.5, -2, 3, pi / 2)
    assert grid_to_world(0, 0, rotated) == pytest.approx((-2.25, 3.25))


@pytest.mark.parametrize("row,col", [(-1, 0), (2, 0), (0, 3)])
def test_grid_bounds(row, col):
    with pytest.raises(ValueError, match="outside"):
        grid_to_world(row, col, GridGeometry(3, 2, 0.5))


@pytest.mark.parametrize("x,y", [(-0.01, 0), (1.5, 0), (0, 1), (float("nan"), 0)])
def test_world_bounds(x, y):
    with pytest.raises(ValueError):
        world_to_grid(x, y, GridGeometry(3, 2, 0.5))


def test_invalid_and_tilted_origin():
    message = map_message([0] * 6)
    message.info.origin.orientation.w = 0.0
    with pytest.raises(ValueError, match="normalized"):
        GridGeometry.from_message(message)
    message.info.origin.orientation.x = sin(0.1)
    message.info.origin.orientation.w = cos(0.1)
    with pytest.raises(ValueError, match="planar"):
        GridGeometry.from_message(message)


def test_classification_excludes_unknown_and_intermediate_values():
    free, occupied, unknown = classify_cells(
        np.array([[-1, 0, 20, 21, 64, 65, 100]]),
    )
    assert free.tolist() == [[False, True, True, False, False, False, False]]
    assert occupied.tolist() == [[False, False, False, False, False, True, True]]
    assert unknown.tolist() == [[True, False, False, True, True, False, False]]


def test_clearance_edges_and_threshold():
    values = np.zeros((9, 9), dtype=np.int8)
    assert safe_candidate_mask(values, 0.1, 0.05).all()
    mask = safe_candidate_mask(values, 0.1, 0.051)
    assert not mask[0].any()
    assert not mask[:, -1].any()
    assert mask[1:-1, 1:-1].all()
    assert not safe_candidate_mask(values, 0.1, 1.0).any()


@pytest.mark.parametrize("blocked_value", [-1, 40, 100])
def test_clearance_never_understates_distance_to_blocked_square(blocked_value):
    values = np.zeros((15, 15), dtype=np.int8)
    values[7, 7] = blocked_value
    resolution, clearance = 0.1, 0.2
    safe = safe_candidate_mask(values, resolution, clearance)
    assert safe.any()
    assert not safe[7, 7]
    for row, col in np.argwhere(safe):
        dx = max(abs(col - 7) - 0.5, 0) * resolution
        dy = max(abs(row - 7) - 0.5, 0) * resolution
        assert sqrt(dx * dx + dy * dy) >= clearance - 1e-12
        assert min(row + 0.5, col + 0.5, 14.5 - row, 14.5 - col) * resolution >= (
            clearance - 1e-12
        )


def test_zero_clearance_still_excludes_nonfree():
    values = np.array([[-1, 0, 40], [100, 20, 0]])
    np.testing.assert_array_equal(
        safe_candidate_mask(values, 0.1, 0),
        [[False, True, False], [False, True, True]],
    )


def test_pair_stays_in_largest_component():
    safe = np.zeros((8, 10), dtype=bool)
    safe[1:3, 1:3] = True
    safe[4:7, 5:9] = True
    geometry = GridGeometry(10, 8, 0.5, -5, -6, pi / 4)
    pair = select_test_positions(safe, geometry, min_separation_m=1.0)
    assert pair == select_test_positions(safe, geometry, 1.0)
    for point in pair:
        row, col = world_to_grid(*point, geometry)
        assert 4 <= row < 7 and 5 <= col < 9
    assert pair[0] != pair[1]


def test_diagonal_cells_are_not_connected():
    with pytest.raises(ValueError, match="two cells"):
        select_test_positions(np.eye(4, dtype=bool), GridGeometry(4, 4, 1))


@pytest.mark.parametrize("mask", [
    np.zeros((3, 3), dtype=bool), np.eye(3, dtype=bool),
])
def test_no_valid_pair(mask):
    with pytest.raises(ValueError):
        select_test_positions(mask, GridGeometry(3, 3, 1))


def test_insufficient_separation_and_invalid_parameters():
    with pytest.raises(ValueError, match="separation"):
        select_test_positions(np.ones((2, 2), dtype=bool), GridGeometry(2, 2, 0.1))
    with pytest.raises(ValueError):
        safe_candidate_mask(np.zeros((2, 2), dtype=int), 0.0)
    with pytest.raises(ValueError):
        safe_candidate_mask(np.zeros((2, 2), dtype=int), 0.1, -0.1)
    with pytest.raises(ValueError):
        classify_cells(np.zeros((2, 2), dtype=int), 65, 20)
