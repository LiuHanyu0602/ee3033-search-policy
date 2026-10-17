#!/usr/bin/env python3

import argparse
import math
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
import yaml


def classify_ros_map(image):
    occupied = image <= 100
    free = image >= 250
    unknown = ~(occupied | free)

    return free, occupied, unknown


def make_memory_image(free_mask, searched_mask):
    """
    Visualization:
      black      = wall/unknown
      light gray = free but not searched
      blue       = searched
    """
    display = np.zeros(
        (*free_mask.shape, 3),
        dtype=np.uint8
    )

    display[free_mask] = [215, 215, 215]
    display[searched_mask & free_mask] = [70, 160, 245]

    return display


def main():
    parser = argparse.ArgumentParser(
        description="Simulate a greedy coverage-memory search"
    )

    default_map = (
        Path(__file__).resolve().parent.parent
        / "outputs"
        / "synthetic_maze_001"
        / "synthetic_maze_001.yaml"
    )

    parser.add_argument(
        "--map-yaml",
        type=Path,
        default=default_map,
    )

    parser.add_argument(
        "--target-coverage",
        type=float,
        default=0.995,
        help="Stop when this fraction of free space is searched",
    )

    args = parser.parse_args()

    if not 0.0 < args.target_coverage <= 1.0:
        raise ValueError(
            "target-coverage must be in (0, 1]"
        )

    yaml_path = args.map_yaml.resolve()

    if not yaml_path.exists():
        raise FileNotFoundError(
            f"Map YAML not found: {yaml_path}"
        )

    with yaml_path.open("r", encoding="utf-8") as file:
        metadata = yaml.safe_load(file)

    image_path = Path(metadata["image"])

    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path

    candidate_path = (
        yaml_path.parent / "candidate_viewpoints.npz"
    )

    image = cv2.imread(
        str(image_path),
        cv2.IMREAD_GRAYSCALE
    )

    if image is None:
        raise RuntimeError(
            f"Cannot read map image: {image_path}"
        )

    if not candidate_path.exists():
        raise FileNotFoundError(
            "candidate_viewpoints.npz was not found. "
            "Run generate_viewpoints.py first."
        )

    free_mask, occupied_mask, unknown_mask = (
        classify_ros_map(image)
    )

    candidate_data = np.load(candidate_path)

    rows = candidate_data["rows"]
    cols = candidate_data["cols"]
    world_x = candidate_data["world_x"]
    world_y = candidate_data["world_y"]
    headings = candidate_data["headings"]

    visibility_masks = candidate_data[
        "visibility_masks"
    ].astype(bool)

    resolution = float(
        candidate_data["resolution"]
    )

    number_of_candidates = len(rows)
    total_free_pixels = int(
        np.count_nonzero(free_mask)
    )

    if total_free_pixels == 0:
        raise RuntimeError("Map has no free space")

    searched_mask = np.zeros_like(
        free_mask,
        dtype=bool
    )

    used_candidates = np.zeros(
        number_of_candidates,
        dtype=bool
    )

    selected_indices = []
    gain_pixels_history = []
    gain_area_history = []
    coverage_history = [0.0]

    # Save states when these percentages are first reached.
    snapshot_targets = [
        0.00,
        0.25,
        0.50,
        0.75,
        0.95,
    ]

    snapshots = [{
        "target": 0.00,
        "step": 0,
        "coverage": 0.0,
        "searched": searched_mask.copy(),
        "candidate_index": None,
    }]

    next_snapshot_index = 1

    for step in range(number_of_candidates):
        unsearched_mask = (
            free_mask & ~searched_mask
        )

        # A_i = number of currently unsearched pixels
        # visible from candidate pose i.
        gains_pixels = np.count_nonzero(
            visibility_masks
            & unsearched_mask[None, :, :],
            axis=(1, 2),
        )

        # Do not choose exactly the same pose again.
        gains_pixels[used_candidates] = -1

        best_index = int(
            np.argmax(gains_pixels)
        )

        best_gain_pixels = int(
            gains_pixels[best_index]
        )

        if best_gain_pixels <= 0:
            print(
                "No candidate can reveal more free space."
            )
            break

        used_candidates[best_index] = True
        selected_indices.append(best_index)
        gain_pixels_history.append(
            best_gain_pixels
        )

        gain_area = (
            best_gain_pixels
            * resolution
            * resolution
        )

        gain_area_history.append(gain_area)

        searched_mask |= (
            visibility_masks[best_index]
            & free_mask
        )

        searched_pixels = int(
            np.count_nonzero(
                searched_mask & free_mask
            )
        )

        coverage = (
            searched_pixels / total_free_pixels
        )

        coverage_history.append(coverage)

        while (
            next_snapshot_index
            < len(snapshot_targets)
            and coverage
            >= snapshot_targets[next_snapshot_index]
        ):
            snapshots.append({
                "target": snapshot_targets[
                    next_snapshot_index
                ],
                "step": step + 1,
                "coverage": coverage,
                "searched": searched_mask.copy(),
                "candidate_index": best_index,
            })

            next_snapshot_index += 1

        print(
            f"Step {step + 1:02d} | "
            f"pose {best_index:03d} | "
            f"gain {gain_area:.3f} m² | "
            f"coverage {coverage * 100.0:.2f}%"
        )

        if coverage >= args.target_coverage:
            break

    final_coverage = coverage_history[-1]

    # If a requested snapshot was not reached, show the final state.
    while len(snapshots) < len(snapshot_targets):
        snapshots.append({
            "target": snapshot_targets[len(snapshots)],
            "step": len(selected_indices),
            "coverage": final_coverage,
            "searched": searched_mask.copy(),
            "candidate_index": (
                selected_indices[-1]
                if selected_indices
                else None
            ),
        })

    selected_indices = np.asarray(
        selected_indices,
        dtype=np.int32
    )

    gain_pixels_history = np.asarray(
        gain_pixels_history,
        dtype=np.int32
    )

    gain_area_history = np.asarray(
        gain_area_history,
        dtype=np.float32
    )

    coverage_history = np.asarray(
        coverage_history,
        dtype=np.float32
    )

    output_data = (
        yaml_path.parent / "coverage_episode.npz"
    )

    output_figure = (
        yaml_path.parent / "coverage_memory.png"
    )

    np.savez_compressed(
        output_data,
        selected_indices=selected_indices,
        gain_pixels=gain_pixels_history,
        gain_area_m2=gain_area_history,
        coverage=coverage_history,
        final_searched_mask=(
            searched_mask.astype(np.uint8)
        ),
    )

    figure, axes = plt.subplots(
        2,
        3,
        figsize=(16, 10)
    )

    map_axes = [
        axes[0, 0],
        axes[0, 1],
        axes[0, 2],
        axes[1, 0],
        axes[1, 1],
    ]

    for axis, snapshot in zip(
        map_axes,
        snapshots
    ):
        axis.imshow(
            make_memory_image(
                free_mask,
                snapshot["searched"],
            )
        )

        candidate_index = snapshot[
            "candidate_index"
        ]

        if candidate_index is not None:
            row = int(rows[candidate_index])
            col = int(cols[candidate_index])
            heading = float(
                headings[candidate_index]
            )

            arrow_length = 8

            axis.arrow(
                col,
                row,
                arrow_length * math.cos(heading),
                -arrow_length * math.sin(heading),
                color="red",
                width=0.7,
                head_width=2.8,
                length_includes_head=True,
            )

        axis.set_title(
            f"Target {snapshot['target'] * 100:.0f}%\n"
            f"step={snapshot['step']}, "
            f"actual={snapshot['coverage'] * 100:.1f}%"
        )

        axis.set_xlabel("pixel column")
        axis.set_ylabel("pixel row")

    curve_axis = axes[1, 2]

    steps = np.arange(
        len(coverage_history)
    )

    curve_axis.plot(
        steps,
        coverage_history * 100.0,
        marker="o",
        markersize=3,
    )

    curve_axis.axhline(
        args.target_coverage * 100.0,
        color="red",
        linestyle="--",
        label="target coverage",
    )

    curve_axis.set_xlabel(
        "Selected candidate poses"
    )

    curve_axis.set_ylabel(
        "Searched free space (%)"
    )

    curve_axis.set_title(
        "Coverage growth"
    )

    curve_axis.grid(True, alpha=0.3)
    curve_axis.legend()

    figure.suptitle(
        "Greedy Coverage Memory Simulation",
        fontsize=16,
    )

    figure.tight_layout()
    figure.savefig(
        output_figure,
        dpi=160
    )

    plt.close(figure)

    total_free_area = (
        total_free_pixels
        * resolution
        * resolution
    )

    searched_area = (
        np.count_nonzero(
            searched_mask & free_mask
        )
        * resolution
        * resolution
    )

    print()
    print("===== COVERAGE MEMORY RESULT =====")
    print(
        f"Available candidate poses: "
        f"{number_of_candidates}"
    )
    print(
        f"Selected candidate poses: "
        f"{len(selected_indices)}"
    )
    print(
        f"Total free area: "
        f"{total_free_area:.3f} m²"
    )
    print(
        f"Searched area: "
        f"{searched_area:.3f} m²"
    )
    print(
        f"Final coverage: "
        f"{final_coverage * 100.0:.2f}%"
    )
    print(f"Saved data: {output_data}")
    print(f"Saved figure: {output_figure}")

    print()
    print("===== FIRST SELECTED POSES =====")

    for order, candidate_index in enumerate(
        selected_indices[:10],
        start=1,
    ):
        print(
            f"{order:02d}: "
            f"pose={candidate_index:03d}, "
            f"x={world_x[candidate_index]:.2f}, "
            f"y={world_y[candidate_index]:.2f}, "
            f"yaw={math.degrees(headings[candidate_index]):.0f} deg, "
            f"gain={gain_area_history[order - 1]:.3f} m²"
        )


if __name__ == "__main__":
    main()