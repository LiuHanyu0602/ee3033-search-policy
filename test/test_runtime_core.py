import hashlib
from pathlib import Path

import numpy as np
import pytest

from ee3033_search_policy.coverage_memory import CoverageMemory
from ee3033_search_policy.feature_builder import (
    build_candidate_features,
    make_shortlist,
)
from ee3033_search_policy.map_utils import GridGeometry, classify_cells, world_to_grid
from ee3033_search_policy.policy_decision import select_with_fallback
from ee3033_search_policy.runtime_viewpoints import (
    generate_viewpoints_from_grid,
    has_line_of_sight,
    raycast_visibility,
    reachable_component,
)


def test_line_of_sight_and_occupancy_grid_y_direction():
    free = np.ones((7, 7), dtype=bool)
    free[3, 4] = False
    assert not has_line_of_sight(free, 3, 2, 3, 6)
    assert has_line_of_sight(free, 3, 2, 5, 2)
    visible = raycast_visibility(free, 3, 2, np.pi / 2, np.deg2rad(40), 4)
    assert visible[5, 2]
    assert not visible[1, 2]


def test_runtime_viewpoints_have_four_orientations_and_world_yaw():
    values = np.zeros((12, 12), dtype=np.int8)
    geometry = GridGeometry(12, 12, 0.1, 1.0, 2.0, np.pi / 2)
    result = generate_viewpoints_from_grid(
        values, geometry, spacing_m=0.4, clearance_m=0.15,
        camera_fov_deg=70.0, camera_range_m=0.5,
    )
    assert len(result) > 0 and len(result) % 4 == 0
    assert result.visibility_masks.shape == (len(result), 12, 12)
    assert np.all(result.clearance_m >= 0.15)
    assert result.clearance_grid_m.shape == values.shape
    assert result.headings[0] == pytest.approx(np.pi / 2)
    assert np.any(result.observable_mask)


def test_runtime_viewpoints_never_use_or_see_unknown_cells():
    values = np.zeros((12, 12), dtype=np.int8)
    values[4:8, 4:8] = -1
    geometry = GridGeometry(12, 12, 0.1)
    result = generate_viewpoints_from_grid(
        values, geometry, spacing_m=0.3, clearance_m=0.15,
        camera_fov_deg=90.0, camera_range_m=1.0,
    )
    free, _, unknown = classify_cells(values)
    assert np.all(free[result.rows, result.cols])
    assert not np.any(unknown[result.rows, result.cols])
    assert not np.any(result.visibility_masks & unknown)
    assert not np.any(result.safe_mask & unknown)


def test_reachable_component_excludes_disconnected_safe_space():
    safe = np.ones((5, 7), dtype=bool)
    safe[:, 3] = False
    reachable = reachable_component(safe, (2, 1))
    assert np.all(reachable[:, :3])
    assert not np.any(reachable[:, 3:])
    assert not np.any(reachable_component(safe, (2, 3)))


def test_runtime_viewpoints_on_tracked_real_map_fixture():
    root = Path(__file__).resolve().parents[1]
    yaml_path = root / "maps" / "balance_map_1.yaml"
    image_path = root / "maps" / "balance_map_1.pgm"
    if not yaml_path.exists() and not image_path.exists():
        pytest.skip("Optional tracked real-map fixture is absent from this workspace")
    assert yaml_path.is_file() and image_path.is_file(), "Real-map fixture is incomplete"
    yaml_bytes, image_bytes = yaml_path.read_bytes(), image_path.read_bytes()
    assert hashlib.sha256(yaml_bytes).hexdigest() == (
        "3abb44528a205eede9f8f003605249bae85bba5b0ee72da478b639f38ff2a42d"
    )
    assert hashlib.sha256(image_bytes).hexdigest() == (
        "dd4bc8600d644bd52817e561d0e4fb7e9947f5700c33f0f0783adf4735984420"
    )
    header = b"P5\n142 160\n255\n"
    assert image_bytes.startswith(header)
    image = np.frombuffer(image_bytes[len(header):], dtype=np.uint8).reshape(160, 142)
    probability = (255.0 - image.astype(float)) / 255.0
    values = np.flipud(np.where(
        probability > 0.65, 100, np.where(probability < 0.25, 0, -1),
    )).astype(np.int8)
    geometry = GridGeometry(142, 160, 0.05, -5.85, -5.49, 0.0)
    free, _, _ = classify_cells(values)
    result = generate_viewpoints_from_grid(values, geometry)

    assert np.count_nonzero(free) == 22298
    assert len(result) == 436
    assert np.count_nonzero(result.observable_mask) == 22230
    assert np.all(free[result.rows, result.cols])
    assert np.all(result.clearance_m >= 0.18)
    assert not np.any(result.visibility_masks & ~free)
    assert np.array_equal(
        result.observable_mask, np.any(result.visibility_masks, axis=0) & free,
    )
    np.testing.assert_allclose(
        result.visible_areas_m2,
        result.visibility_masks.sum(axis=(1, 2)) * geometry.resolution ** 2,
        rtol=1e-5, atol=1e-6,
    )
    for row, col, x, y in zip(result.rows, result.cols, result.world_x, result.world_y):
        assert world_to_grid(float(x), float(y), geometry) == (int(row), int(col))


def test_coverage_updates_only_after_completed_observation(tmp_path):
    observable = np.ones((3, 4), dtype=bool)
    masks = np.zeros((2, 3, 4), dtype=bool)
    masks[0, :, :2] = True
    masks[1, :, 2:] = True
    memory = CoverageMemory(observable, 2)
    assert memory.coverage_fraction == 0.0
    memory.pause()
    assert not memory.mark_observation_complete(0, masks[0])
    assert memory.coverage_fraction == 0.0
    memory.resume()
    assert memory.mark_observation_complete(0, masks[0])
    assert memory.coverage_fraction == pytest.approx(0.5)
    assert memory.gains(masks).tolist() == [0, 6]

    path = tmp_path / "memory.npz"
    memory.save(path)
    loaded = CoverageMemory.load(path, observable)
    assert np.array_equal(loaded.searched_mask, memory.searched_mask)
    assert loaded.visit_counts.tolist() == [1, 0]


def test_shortlist_and_feature_order_match_training_formula():
    gains = np.array([100, 80, 0, 50])
    distances = np.array([10.0, 1.0, 0.5, np.inf])
    turns = np.array([0.0, 0.4, 0.0, 0.0])
    shortlist, times, utility = make_shortlist(
        gains, distances, turns, shortlist_size=2,
    )
    assert shortlist.tolist() == [1, 0]
    assert np.isneginf(utility[2]) and np.isneginf(utility[3])

    features = build_candidate_features(
        shortlist,
        gain_counts=gains,
        unseen_count=200,
        searched_count=300,
        path_distances_m=distances,
        turn_angles_rad=turns,
        clearance_m=np.array([0.3, 0.4, 0.5, 0.6]),
        visit_counts=np.array([2, 1, 0, 0]),
        visible_areas_m2=np.array([1.2, 1.0, 0.5, 0.8]),
        visible_counts=np.array([120, 100, 20, 80]),
        euclidean_distances_m=np.array([8.0, 0.8, 0.5, 2.0]),
        resolution=0.05,
    )
    assert features.shape == (2, 12)
    # First shortlisted candidate is original candidate 1.
    assert features[0, 0] == pytest.approx(80 * 0.05 ** 2)
    assert features[0, 1] == pytest.approx(80 / 200)
    assert features[0, 7] == pytest.approx(300 / 500)
    assert features[0, 9] == pytest.approx((100 - 80) / 100)
    assert features[0, 11] == pytest.approx(1.0 / 0.8)


def test_policy_falls_back_when_model_rejects_features():
    class BrokenPolicy:
        def select(self, features):
            raise ValueError("bad model")

    selected, scores, source = select_with_fallback(
        BrokenPolicy(), np.zeros((2, 12)), np.array([4, 1]),
        np.array([0.0, 0.7, 0.2, 0.1, 0.9]),
    )
    assert selected == 4
    assert source == "fallback"
    assert scores.tolist() == pytest.approx([0.9, 0.7])
