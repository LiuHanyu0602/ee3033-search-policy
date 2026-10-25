import math

import numpy as np
import pytest

from ee3033_search_policy.target_localization_core import (
    CameraModel,
    bbox_contact_pixel,
    choose_gated_range,
    pixel_to_camera_ray,
    ransac_triangulate,
    ray_ground_intersection,
    robust_lidar_range,
    robust_weighted_fusion,
    triangulate_bearings,
)


def test_contact_pixel_uses_bbox_bottom_not_centre():
    assert bbox_contact_pixel(320, 200, 40, 100, 480) == (320.0, 250.0)


def test_principal_point_ray_is_optical_axis():
    camera = CameraModel(500.0, 500.0, 320.0, 240.0)
    assert pixel_to_camera_ray(320, 240, camera) == pytest.approx([0, 0, 1])


def test_ground_intersection_with_known_rotation():
    # Optical +z points base +x; optical +y points base -z.
    rotation = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)
    ray = np.array([0.0, 0.5, 1.0])
    ray /= np.linalg.norm(ray)
    point = ray_ground_intersection(ray, rotation, [0, 0, 1.0])
    assert point == pytest.approx([2.0, 0.0, 0.0])


def test_lidar_median_rejects_outlier_and_wall_return():
    angles = np.linspace(-0.1, 0.1, 7)
    ranges = [2.02, 1.98, 2.01, 8.0, 2.00, np.inf, 2.03]
    measured = robust_lidar_range(ranges, angles, 0.0, 0.11, 0.1, 10.0)
    assert measured == pytest.approx(2.01, abs=0.02)
    distance, method = choose_gated_range(1.8, 4.5, gate_m=0.3)
    assert distance == pytest.approx(1.8)
    assert method == "ground_lidar_rejected"


def test_weighted_triangulation_recovers_target():
    target = np.array([2.0, 1.0])
    origins = np.array([[0, 0], [0.6, 0], [0, 0.7]], dtype=float)
    directions = target - origins
    estimate = triangulate_bearings(origins, directions)
    assert estimate == pytest.approx(target)


def test_ransac_rejects_bad_bearing():
    target = np.array([2.0, 1.0])
    origins = np.array([[0, 0], [0.7, 0], [0, 0.8], [0.8, 0.8]], dtype=float)
    directions = target - origins
    directions[-1] = [1.0, -1.0]
    estimate, inliers = ransac_triangulate(
        origins, directions, threshold_m=0.05, iterations=200
    )
    assert estimate == pytest.approx(target)
    assert inliers.tolist() == [True, True, True, False]


def test_robust_weighted_fusion_rejects_far_point():
    points = np.array([[1.0, 2.0], [1.1, 2.0], [0.9, 2.1], [8.0, -4.0]])
    mean, covariance, inliers = robust_weighted_fusion(points, gate_m=0.3)
    assert inliers.tolist() == [True, True, True, False]
    assert mean == pytest.approx([1.0, 2.033333333])
    assert np.all(np.isfinite(covariance))
