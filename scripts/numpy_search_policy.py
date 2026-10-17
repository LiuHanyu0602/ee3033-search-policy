#!/usr/bin/env python3
"""Pure NumPy inference for the EE3033 Tiny Search Policy V2."""

import argparse
import hashlib
import time
from pathlib import Path

import numpy as np


EXPECTED_FEATURE_NAMES = (
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

EXPECTED_V2_SHA256 = (
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


class ModelValidationError(ValueError):
    """Raised when a model or feature matrix is unsafe to use."""


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as file:
        for block in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


class TinySearchPolicy:
    """Validated 12-32-16-1 MLP implemented with NumPy only."""

    def __init__(
        self,
        weights_path,
        expected_sha256=EXPECTED_V2_SHA256,
    ):
        self.weights_path = Path(weights_path).expanduser().resolve()
        if not self.weights_path.is_file():
            raise FileNotFoundError(
                f"Model file not found: {self.weights_path}"
            )

        self.sha256 = sha256_file(self.weights_path)
        if expected_sha256 and self.sha256 != expected_sha256:
            raise ModelValidationError(
                "Model SHA256 mismatch. "
                f"Expected {expected_sha256}, got {self.sha256}."
            )

        with np.load(self.weights_path, allow_pickle=False) as data:
            missing = set(REQUIRED_SHAPES) - set(data.files)
            if missing:
                raise ModelValidationError(
                    f"Model is missing arrays: {sorted(missing)}"
                )
            arrays = {
                key: data[key].copy()
                for key in REQUIRED_SHAPES
            }

        for key, expected_shape in REQUIRED_SHAPES.items():
            if arrays[key].shape != expected_shape:
                raise ModelValidationError(
                    f"{key} has shape {arrays[key].shape}; "
                    f"expected {expected_shape}."
                )

        names = tuple(str(item) for item in arrays["feature_names"])
        if names != EXPECTED_FEATURE_NAMES:
            raise ModelValidationError(
                "Feature order mismatch.\n"
                f"Expected: {EXPECTED_FEATURE_NAMES}\n"
                f"Received: {names}"
            )

        numeric_keys = [
            key for key in REQUIRED_SHAPES
            if key != "feature_names"
        ]
        for key in numeric_keys:
            array = np.asarray(arrays[key], dtype=np.float32)
            if not np.all(np.isfinite(array)):
                raise ModelValidationError(
                    f"{key} contains NaN or infinity."
                )
            arrays[key] = array

        if np.any(arrays["feature_std"] <= 0.0):
            raise ModelValidationError(
                "feature_std must contain only positive values."
            )

        self.feature_names = names
        self.feature_mean = arrays["feature_mean"]
        self.feature_std = arrays["feature_std"]
        self.weight_1 = arrays["weight_1"]
        self.bias_1 = arrays["bias_1"]
        self.weight_2 = arrays["weight_2"]
        self.bias_2 = arrays["bias_2"]
        self.weight_3 = arrays["weight_3"]
        self.bias_3 = arrays["bias_3"]

    def score(self, features):
        """Return one score per candidate feature row."""
        feature_matrix = np.asarray(features, dtype=np.float32)
        input_was_vector = feature_matrix.ndim == 1

        if input_was_vector:
            feature_matrix = feature_matrix.reshape(1, -1)

        if feature_matrix.ndim != 2:
            raise ModelValidationError(
                "features must have shape (12,) or (N, 12)."
            )
        if feature_matrix.shape[1] != len(EXPECTED_FEATURE_NAMES):
            raise ModelValidationError(
                f"Expected 12 features, got {feature_matrix.shape[1]}."
            )
        if feature_matrix.shape[0] == 0:
            raise ModelValidationError("No candidate features supplied.")
        if not np.all(np.isfinite(feature_matrix)):
            raise ModelValidationError(
                "Candidate features contain NaN or infinity."
            )

        normalized = (
            feature_matrix - self.feature_mean
        ) / self.feature_std

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
            raise ModelValidationError(
                "Model output contains NaN or infinity."
            )

        if input_was_vector:
            return float(scores[0])
        return scores

    def select(self, features, valid_mask=None):
        """Return (selected_index, all_scores) for valid candidates."""
        scores = np.asarray(self.score(features), dtype=np.float32)
        if scores.ndim == 0:
            scores = scores.reshape(1)

        if valid_mask is None:
            valid = np.ones(scores.shape, dtype=bool)
        else:
            valid = np.asarray(valid_mask, dtype=bool)
            if valid.shape != scores.shape:
                raise ModelValidationError(
                    f"valid_mask shape {valid.shape} does not match "
                    f"score shape {scores.shape}."
                )

        if not np.any(valid):
            raise ModelValidationError(
                "No valid candidate is available for selection."
            )

        masked_scores = np.where(valid, scores, -np.inf)
        selected_index = int(np.argmax(masked_scores))
        return selected_index, scores


def generate_reference_features(policy, candidate_count, seed):
    """Generate deterministic, finite test features around training scale."""
    random_generator = np.random.default_rng(seed)
    standard_values = random_generator.normal(
        loc=0.0,
        scale=1.0,
        size=(candidate_count, len(EXPECTED_FEATURE_NAMES)),
    ).astype(np.float32)
    return (
        policy.feature_mean[None, :]
        + standard_values * policy.feature_std[None, :]
    ).astype(np.float32)


def write_reference(path, policy, features, scores):
    output_path = Path(path).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        output_path,
        features=features.astype(np.float32),
        scores=np.asarray(scores, dtype=np.float32),
        feature_names=np.asarray(policy.feature_names),
        model_sha256=np.asarray(policy.sha256),
    )
    return output_path


def check_reference(path, policy):
    reference_path = Path(path).expanduser().resolve()
    with np.load(reference_path, allow_pickle=False) as data:
        required = {
            "features",
            "scores",
            "feature_names",
            "model_sha256",
        }
        missing = required - set(data.files)
        if missing:
            raise ModelValidationError(
                f"Reference file is missing: {sorted(missing)}"
            )
        features = data["features"].astype(np.float32)
        expected_scores = data["scores"].astype(np.float32)
        names = tuple(str(item) for item in data["feature_names"])
        model_sha256 = str(data["model_sha256"])

    if names != policy.feature_names:
        raise ModelValidationError(
            "Reference feature names do not match the model."
        )
    if model_sha256 != policy.sha256:
        raise ModelValidationError(
            "Reference was generated for a different model SHA256."
        )

    actual_scores = np.asarray(policy.score(features), dtype=np.float32)
    if actual_scores.shape != expected_scores.shape:
        raise ModelValidationError(
            "Reference and current score shapes do not match."
        )

    maximum_error = float(
        np.max(np.abs(actual_scores - expected_scores))
    )
    passed = np.allclose(
        actual_scores,
        expected_scores,
        rtol=1.0e-5,
        atol=1.0e-5,
    )
    if not passed:
        raise ModelValidationError(
            "Reference inference check failed; "
            f"maximum absolute error={maximum_error:.3e}."
        )
    return maximum_error


def benchmark(policy, features, iterations):
    for _ in range(100):
        policy.score(features)

    samples_ms = np.empty(iterations, dtype=np.float64)
    for index in range(iterations):
        start_ns = time.perf_counter_ns()
        policy.score(features)
        samples_ms[index] = (
            time.perf_counter_ns() - start_ns
        ) / 1.0e6

    return {
        "mean_ms": float(np.mean(samples_ms)),
        "median_ms": float(np.median(samples_ms)),
        "p95_ms": float(np.percentile(samples_ms, 95.0)),
        "maximum_ms": float(np.max(samples_ms)),
    }


def default_weights_path():
    return (
        Path(__file__).resolve().parent.parent
        / "models"
        / "tiny_search_policy_weights.npz"
    )


def main():
    parser = argparse.ArgumentParser(
        description="Validate and benchmark Tiny Search Policy V2"
    )
    parser.add_argument(
        "--weights",
        type=Path,
        default=default_weights_path(),
    )
    parser.add_argument("--candidates", type=int, default=32)
    parser.add_argument("--iterations", type=int, default=10000)
    parser.add_argument("--seed", type=int, default=3033)
    parser.add_argument("--write-reference", type=Path)
    parser.add_argument("--check-reference", type=Path)
    parser.add_argument(
        "--allow-other-model",
        action="store_true",
        help="Validate structure but do not enforce the frozen V2 SHA256",
    )
    args = parser.parse_args()

    if args.candidates < 1:
        raise ValueError("candidates must be positive")
    if args.iterations < 1:
        raise ValueError("iterations must be positive")

    expected_sha256 = None if args.allow_other_model else EXPECTED_V2_SHA256
    policy = TinySearchPolicy(args.weights, expected_sha256)

    print("===== MODEL VALIDATION =====")
    print("Weights:", policy.weights_path)
    print("SHA256:", policy.sha256)
    print("Architecture: 12 -> 32 -> 16 -> 1")
    print("Parameters: 961")
    print("Features:")
    for index, name in enumerate(policy.feature_names):
        print(f"  {index:02d}: {name}")

    features = generate_reference_features(
        policy,
        args.candidates,
        args.seed,
    )
    selected_index, scores = policy.select(features)

    print("\n===== DETERMINISTIC SELF-TEST =====")
    print("Candidate count:", len(scores))
    print("Selected index:", selected_index)
    print("Selected score:", f"{scores[selected_index]:.8f}")
    print("Score range:", f"{scores.min():.8f} to {scores.max():.8f}")
    print("Finite outputs:", bool(np.all(np.isfinite(scores))))

    if args.write_reference:
        reference_path = write_reference(
            args.write_reference,
            policy,
            features,
            scores,
        )
        print("Reference written:", reference_path)

    if args.check_reference:
        maximum_error = check_reference(args.check_reference, policy)
        print(
            "Reference check: PASSED, maximum error "
            f"{maximum_error:.3e}"
        )

    timing = benchmark(policy, features, args.iterations)
    print("\n===== NUMPY INFERENCE BENCHMARK =====")
    print("Iterations:", args.iterations)
    print("Candidates per inference:", args.candidates)
    print("Mean:", f"{timing['mean_ms']:.6f} ms")
    print("Median:", f"{timing['median_ms']:.6f} ms")
    print("P95:", f"{timing['p95_ms']:.6f} ms")
    print("Maximum:", f"{timing['maximum_ms']:.6f} ms")
    print("\nDeployment validation: PASSED")


if __name__ == "__main__":
    main()
