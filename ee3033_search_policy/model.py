"""Validated NumPy runtime for the frozen Tiny Search Policy V2."""

import hashlib
from pathlib import Path

import numpy as np


FEATURE_NAMES = (
    "gain_area_m2",
    "gain_fraction",
    "path_distance_m",
    "turn_angle_rad",
    "travel_time_s",
    "clearance_m",
    "visit_count",
    "coverage_fraction",
    "visible_area_m2",
    "redundancy_fraction",
    "euclidean_distance_m",
    "path_detour_ratio",
)

MODEL_SHA256 = (
    "3f2c9e784f71d3d5921f8e699d0013ca"
    "377034ff70d6f87722b73062d132a3b5"
)

REQUIRED_SHAPES = {
    "feature_mean": (12,),
    "feature_std": (12,),
    "feature_names": (12,),
    "weight_1": (32, 12),
    "bias_1": (32,),
    "weight_2": (16, 32),
    "bias_2": (16,),
    "weight_3": (1, 16),
    "bias_3": (1,),
}


def file_sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


class TinySearchPolicy:
    """Small 12-32-16-1 MLP implemented using NumPy only."""

    def __init__(self, weights_path):
        self.weights_path = Path(weights_path).expanduser().resolve()
        if not self.weights_path.is_file():
            raise FileNotFoundError(self.weights_path)

        self.sha256 = file_sha256(self.weights_path)
        if self.sha256 != MODEL_SHA256:
            raise ValueError("Unexpected model SHA256: " + self.sha256)

        with np.load(self.weights_path, allow_pickle=False) as data:
            missing = set(REQUIRED_SHAPES) - set(data.files)
            if missing:
                raise ValueError(f"Missing model arrays: {sorted(missing)}")
            arrays = {key: data[key].copy() for key in REQUIRED_SHAPES}

        for key, expected_shape in REQUIRED_SHAPES.items():
            if arrays[key].shape != expected_shape:
                raise ValueError(
                    f"{key}: expected {expected_shape}, "
                    f"got {arrays[key].shape}"
                )

        names = tuple(str(value) for value in arrays["feature_names"])
        if names != FEATURE_NAMES:
            raise ValueError("Model feature order does not match runtime")

        self.feature_mean = arrays["feature_mean"].astype(np.float32)
        self.feature_std = arrays["feature_std"].astype(np.float32)
        self.weight_1 = arrays["weight_1"].astype(np.float32)
        self.bias_1 = arrays["bias_1"].astype(np.float32)
        self.weight_2 = arrays["weight_2"].astype(np.float32)
        self.bias_2 = arrays["bias_2"].astype(np.float32)
        self.weight_3 = arrays["weight_3"].astype(np.float32)
        self.bias_3 = arrays["bias_3"].astype(np.float32)

        numeric_arrays = (
            self.feature_mean,
            self.feature_std,
            self.weight_1,
            self.bias_1,
            self.weight_2,
            self.bias_2,
            self.weight_3,
            self.bias_3,
        )
        if not all(np.all(np.isfinite(array)) for array in numeric_arrays):
            raise ValueError("Model contains NaN or infinity")
        if np.any(self.feature_std <= 0.0):
            raise ValueError("Feature standard deviations must be positive")

    def score(self, features):
        values = np.asarray(features, dtype=np.float32)
        if values.ndim != 2 or values.shape[1] != 12:
            raise ValueError("Candidate features must have shape (N, 12)")
        if values.shape[0] == 0 or not np.all(np.isfinite(values)):
            raise ValueError("Candidate features are empty or non-finite")

        normalized = (values - self.feature_mean) / self.feature_std
        hidden_1 = np.maximum(
            normalized @ self.weight_1.T + self.bias_1,
            0.0,
        )
        hidden_2 = np.maximum(
            hidden_1 @ self.weight_2.T + self.bias_2,
            0.0,
        )
        scores = (
            hidden_2 @ self.weight_3.T + self.bias_3
        ).reshape(-1)
        if not np.all(np.isfinite(scores)):
            raise ValueError("Model produced non-finite scores")
        return scores

    def select(self, features, valid_mask=None):
        scores = self.score(features)
        if valid_mask is None:
            valid = np.ones(scores.shape, dtype=bool)
        else:
            valid = np.asarray(valid_mask, dtype=bool)
            if valid.shape != scores.shape:
                raise ValueError("valid_mask has the wrong shape")
        if not np.any(valid):
            raise ValueError("No valid candidate viewpoint")
        masked = np.where(valid, scores, -np.inf)
        return int(np.argmax(masked)), scores
