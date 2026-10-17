#!/usr/bin/env python3

import argparse
import heapq
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


def wrap_angle(angle):
    return (
        angle + math.pi
    ) % (2.0 * math.pi) - math.pi


def dijkstra_distance_field(
    safe_mask,
    start_row,
    start_col,
    save_parents=False,
):
    """
    Calculate shortest-path distance from the current position
    to every safe map pixel.

    Distances are returned in pixel units.
    """
    height, width = safe_mask.shape

    distances = np.full(
        (height, width),
        np.inf,
        dtype=np.float64,
    )

    if save_parents:
        parent_rows = np.full(
            (height, width),
            -1,
            dtype=np.int32,
        )

        parent_cols = np.full(
            (height, width),
            -1,
            dtype=np.int32,
        )
    else:
        parent_rows = None
        parent_cols = None

    if not safe_mask[start_row, start_col]:
        return distances, parent_rows, parent_cols

    distances[start_row, start_col] = 0.0

    queue = [
        (0.0, int(start_row), int(start_col))
    ]

    neighbours = [
        (-1, 0, 1.0),
        (1, 0, 1.0),
        (0, -1, 1.0),
        (0, 1, 1.0),
        (-1, -1, math.sqrt(2.0)),
        (-1, 1, math.sqrt(2.0)),
        (1, -1, math.sqrt(2.0)),
        (1, 1, math.sqrt(2.0)),
    ]

    while queue:
        current_distance, row, col = heapq.heappop(
            queue
        )

        if (
            current_distance
            > float(distances[row, col]) + 1e-12
        ):
            continue

        for dr, dc, move_cost in neighbours:
            next_row = row + dr
            next_col = col + dc

            if not (
                0 <= next_row < height
                and 0 <= next_col < width
            ):
                continue

            if not safe_mask[next_row, next_col]:
                continue

            # Prevent diagonal corner cutting.
            if dr != 0 and dc != 0:
                if not (
                    safe_mask[row, next_col]
                    and safe_mask[next_row, col]
                ):
                    continue

            new_distance = (
                current_distance + move_cost
            )

            if (
                new_distance
                < distances[next_row, next_col]
            ):
                distances[next_row, next_col] = (
                    new_distance
                )

                if save_parents:
                    parent_rows[
                        next_row, next_col
                    ] = row

                    parent_cols[
                        next_row, next_col
                    ] = col

                heapq.heappush(
                    queue,
                    (
                        new_distance,
                        next_row,
                        next_col,
                    ),
                )

    return distances, parent_rows, parent_cols


def reconstruct_path(
    parent_rows,
    parent_cols,
    start_row,
    start_col,
    goal_row,
    goal_col,
):
    path = [
        (int(goal_row), int(goal_col))
    ]

    row = int(goal_row)
    col = int(goal_col)

    maximum_iterations = (
        parent_rows.shape[0]
        * parent_rows.shape[1]
    )

    for _ in range(maximum_iterations):
        if row == start_row and col == start_col:
            path.reverse()
            return path

        previous_row = int(
            parent_rows[row, col]
        )

        previous_col = int(
            parent_cols[row, col]
        )

        if previous_row < 0 or previous_col < 0:
            return []

        row = previous_row
        col = previous_col
        path.append((row, col))

    return []


def normalise_valid(values, valid_mask):
    """
    Normalize valid finite values into [0, 1].
    """
    output = np.zeros_like(
        values,
        dtype=np.float32
    )

    valid_values = values[valid_mask]

    if len(valid_values) == 0:
        return output

    maximum = float(np.max(valid_values))

    if maximum > 1e-9:
        output[valid_mask] = (
            valid_values / maximum
        )

    return output


def evaluate_sequence_distance(
    sequence,
    start_index,
    rows,
    cols,
    headings,
    safe_mask,
    resolution,
):
    """
    Measure route length and total rotation for an existing
    candidate sequence.
    """
    current_row = int(rows[start_index])
    current_col = int(cols[start_index])
    current_heading = float(
        headings[start_index]
    )

    cumulative_distances = [0.0]
    total_distance = 0.0
    total_rotation = 0.0

    for candidate_index in sequence:
        candidate_index = int(candidate_index)

        distance_field, _, _ = (
            dijkstra_distance_field(
                safe_mask,
                current_row,
                current_col,
                save_parents=False,
            )
        )

        goal_row = int(rows[candidate_index])
        goal_col = int(cols[candidate_index])

        distance_pixels = float(
            distance_field[goal_row, goal_col]
        )

        if not np.isfinite(distance_pixels):
            return np.inf, np.inf, cumulative_distances

        distance_metres = (
            distance_pixels * resolution
        )

        rotation = abs(
            wrap_angle(
                float(headings[candidate_index])
                - current_heading
            )
        )

        total_distance += distance_metres
        total_rotation += rotation
        cumulative_distances.append(
            total_distance
        )

        current_row = goal_row
        current_col = goal_col
        current_heading = float(
            headings[candidate_index]
        )

    return (
        total_distance,
        total_rotation,
        cumulative_distances,
    )


def main():
    parser = argparse.ArgumentParser(
        description="Simulate path-aware visual search"
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
        "--start-pose-index",
        type=int,
        default=0,
    )

    parser.add_argument(
        "--target-coverage",
        type=float,
        default=0.995,
    )

    parser.add_argument(
        "--weight-gain",
        type=float,
        default=1.00,
    )

    parser.add_argument(
        "--weight-distance",
        type=float,
        default=0.30,
    )

    parser.add_argument(
        "--weight-turn",
        type=float,
        default=0.10,
    )

    parser.add_argument(
        "--weight-visit",
        type=float,
        default=0.08,
    )

    parser.add_argument(
        "--weight-clearance",
        type=float,
        default=0.05,
    )

    args = parser.parse_args()

    yaml_path = args.map_yaml.resolve()

    with yaml_path.open("r", encoding="utf-8") as file:
        metadata = yaml.safe_load(file)

    image_path = Path(metadata["image"])

    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path

    candidate_path = (
        yaml_path.parent / "candidate_viewpoints.npz"
    )

    navigation_path = (
        yaml_path.parent / "navigation_mask.png"
    )

    clearance_path = (
        yaml_path.parent / "clearance_metres.npy"
    )

    coverage_baseline_path = (
        yaml_path.parent / "coverage_episode.npz"
    )

    image = cv2.imread(
        str(image_path),
        cv2.IMREAD_GRAYSCALE,
    )

    navigation_image = cv2.imread(
        str(navigation_path),
        cv2.IMREAD_GRAYSCALE,
    )

    if image is None or navigation_image is None:
        raise RuntimeError(
            "Map or navigation mask could not be loaded"
        )

    if not candidate_path.exists():
        raise FileNotFoundError(
            "candidate_viewpoints.npz was not found"
        )

    free_mask, _, _ = classify_ros_map(image)
    safe_mask = navigation_image >= 250
    clearance_m = np.load(clearance_path)

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

    if not (
        0 <= args.start_pose_index
        < number_of_candidates
    ):
        raise ValueError(
            "start-pose-index is outside candidate range"
        )

    current_row = int(
        rows[args.start_pose_index]
    )

    current_col = int(
        cols[args.start_pose_index]
    )

    current_heading = float(
        headings[args.start_pose_index]
    )

    searched_mask = np.zeros_like(
        free_mask,
        dtype=bool,
    )

    used_candidates = np.zeros(
        number_of_candidates,
        dtype=bool,
    )

    position_visits = np.zeros_like(
        free_mask,
        dtype=np.int32,
    )

    total_free_pixels = int(
        np.count_nonzero(free_mask)
    )

    candidate_clearances = clearance_m[
        rows, cols
    ]

    clearance_normalized = np.clip(
        candidate_clearances / 0.50,
        0.0,
        1.0,
    )

    selected_indices = []
    coverage_history = [0.0]
    cumulative_distance = [0.0]
    gain_area_history = []
    step_distance_history = []
    turn_history = []
    score_history = []
    route_segments = []

    total_route_distance = 0.0
    total_rotation = 0.0

    for step in range(number_of_candidates):
        unsearched_mask = (
            free_mask & ~searched_mask
        )

        gain_pixels = np.count_nonzero(
            visibility_masks
            & unsearched_mask[None, :, :],
            axis=(1, 2),
        ).astype(np.float32)

        gain_area = (
            gain_pixels
            * resolution
            * resolution
        )

        (
            distance_field,
            parent_rows,
            parent_cols,
        ) = dijkstra_distance_field(
            safe_mask,
            current_row,
            current_col,
            save_parents=True,
        )

        candidate_distance_pixels = (
            distance_field[rows, cols]
        )

        candidate_distance_metres = (
            candidate_distance_pixels
            * resolution
        )

        turn_angles = np.abs(
            np.arctan2(
                np.sin(headings - current_heading),
                np.cos(headings - current_heading),
            )
        )

        visit_values = position_visits[
            rows, cols
        ].astype(np.float32)

        valid = (
            ~used_candidates
            & (gain_pixels > 0)
            & np.isfinite(
                candidate_distance_metres
            )
        )

        if not np.any(valid):
            print(
                "No reachable candidate can reveal "
                "additional free space."
            )
            break

        gain_normalized = normalise_valid(
            gain_area,
            valid,
        )

        distance_normalized = normalise_valid(
            candidate_distance_metres,
            valid,
        )

        turn_normalized = (
            turn_angles / math.pi
        ).astype(np.float32)

        visit_normalized = np.clip(
            visit_values / 4.0,
            0.0,
            1.0,
        )

        scores = (
            args.weight_gain
            * gain_normalized
            - args.weight_distance
            * distance_normalized
            - args.weight_turn
            * turn_normalized
            - args.weight_visit
            * visit_normalized
            + args.weight_clearance
            * clearance_normalized
        )

        scores[~valid] = -np.inf

        best_index = int(
            np.argmax(scores)
        )

        goal_row = int(rows[best_index])
        goal_col = int(cols[best_index])

        path = reconstruct_path(
            parent_rows,
            parent_cols,
            current_row,
            current_col,
            goal_row,
            goal_col,
        )

        if not path:
            used_candidates[best_index] = True
            continue

        step_distance = float(
            candidate_distance_metres[best_index]
        )

        step_turn = float(
            turn_angles[best_index]
        )

        selected_indices.append(best_index)
        route_segments.append(path)

        used_candidates[best_index] = True
        position_visits[goal_row, goal_col] += 1

        searched_mask |= (
            visibility_masks[best_index]
            & free_mask
        )

        total_route_distance += step_distance
        total_rotation += step_turn

        total_searched = int(
            np.count_nonzero(
                searched_mask & free_mask
            )
        )

        coverage = (
            total_searched / total_free_pixels
        )

        coverage_history.append(coverage)
        cumulative_distance.append(
            total_route_distance
        )

        gain_area_history.append(
            float(gain_area[best_index])
        )

        step_distance_history.append(
            step_distance
        )

        turn_history.append(step_turn)
        score_history.append(
            float(scores[best_index])
        )

        print(
            f"Step {step + 1:02d} | "
            f"pose {best_index:03d} | "
            f"gain {gain_area[best_index]:.3f} m² | "
            f"path {step_distance:.2f} m | "
            f"turn {math.degrees(step_turn):.0f} deg | "
            f"coverage {coverage * 100.0:.2f}%"
        )

        current_row = goal_row
        current_col = goal_col
        current_heading = float(
            headings[best_index]
        )

        if coverage >= args.target_coverage:
            break

    selected_indices = np.asarray(
        selected_indices,
        dtype=np.int32,
    )

    coverage_history = np.asarray(
        coverage_history,
        dtype=np.float32,
    )

    cumulative_distance = np.asarray(
        cumulative_distance,
        dtype=np.float32,
    )

    gain_area_history = np.asarray(
        gain_area_history,
        dtype=np.float32,
    )

    step_distance_history = np.asarray(
        step_distance_history,
        dtype=np.float32,
    )

    turn_history = np.asarray(
        turn_history,
        dtype=np.float32,
    )

    score_history = np.asarray(
        score_history,
        dtype=np.float32,
    )

    output_data = (
        yaml_path.parent
        / "path_aware_episode.npz"
    )

    output_figure = (
        yaml_path.parent
        / "path_aware_search.png"
    )

    np.savez_compressed(
        output_data,
        selected_indices=selected_indices,
        coverage=coverage_history,
        cumulative_distance_m=cumulative_distance,
        gain_area_m2=gain_area_history,
        step_distance_m=step_distance_history,
        turn_radians=turn_history,
        scores=score_history,
        final_searched_mask=(
            searched_mask.astype(np.uint8)
        ),
    )

    # ---------------------------------------------------------
    # Evaluate the previous coverage-only sequence from the same
    # starting pose, allowing a route-distance comparison.
    # ---------------------------------------------------------
    baseline_total_distance = None
    baseline_total_rotation = None
    baseline_distances = None
    baseline_coverage = None

    if coverage_baseline_path.exists():
        baseline_data = np.load(
            coverage_baseline_path
        )

        baseline_sequence = baseline_data[
            "selected_indices"
        ]

        baseline_coverage = baseline_data[
            "coverage"
        ]

        (
            baseline_total_distance,
            baseline_total_rotation,
            baseline_distances,
        ) = evaluate_sequence_distance(
            sequence=baseline_sequence,
            start_index=args.start_pose_index,
            rows=rows,
            cols=cols,
            headings=headings,
            safe_mask=safe_mask,
            resolution=resolution,
        )

        baseline_distances = np.asarray(
            baseline_distances,
            dtype=np.float32,
        )

    # ---------------------------------------------------------
    # Visualization
    # ---------------------------------------------------------
    display = np.zeros(
        (*free_mask.shape, 3),
        dtype=np.uint8,
    )

    display[free_mask] = [210, 210, 210]
    display[searched_mask] = [80, 160, 245]

    # Actual shortest paths are shown in yellow.
    for path in route_segments:
        path_array = np.asarray(path)

        display[
            path_array[:, 0],
            path_array[:, 1],
        ] = [255, 190, 40]

    figure, axes = plt.subplots(
        1,
        3,
        figsize=(18, 5),
    )

    axes[0].imshow(display)

    if len(selected_indices) > 0:
        axes[0].scatter(
            cols[selected_indices],
            rows[selected_indices],
            c="red",
            s=18,
            label="selected viewpoints",
        )

    axes[0].scatter(
        cols[args.start_pose_index],
        rows[args.start_pose_index],
        c="lime",
        edgecolors="black",
        s=80,
        marker="*",
        label="start",
    )

    axes[0].set_title(
        "Path-aware searched area and route"
    )
    axes[0].set_xlabel("pixel column")
    axes[0].set_ylabel("pixel row")
    axes[0].legend(loc="lower right")

    axes[1].plot(
        cumulative_distance,
        coverage_history * 100.0,
        marker="o",
        markersize=3,
        label="path-aware score",
    )

    if (
        baseline_distances is not None
        and baseline_coverage is not None
    ):
        comparable_length = min(
            len(baseline_distances),
            len(baseline_coverage),
        )

        axes[1].plot(
            baseline_distances[:comparable_length],
            baseline_coverage[:comparable_length]
            * 100.0,
            linestyle="--",
            label="coverage-only greedy",
        )

    axes[1].set_xlabel(
        "Cumulative path length (m)"
    )
    axes[1].set_ylabel(
        "Searched free space (%)"
    )
    axes[1].set_title(
        "Coverage versus travelled distance"
    )
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()

    steps = np.arange(
        1,
        len(selected_indices) + 1,
    )

    axes[2].bar(
        steps,
        gain_area_history,
        alpha=0.7,
        label="new visible area",
    )

    second_axis = axes[2].twinx()

    second_axis.plot(
        steps,
        step_distance_history,
        color="red",
        marker="o",
        markersize=3,
        label="path distance",
    )

    axes[2].set_xlabel("Decision step")
    axes[2].set_ylabel(
        "New visible area (m²)"
    )
    second_axis.set_ylabel(
        "Travel distance (m)",
        color="red",
    )

    axes[2].set_title(
        "Information gain and travel cost"
    )

    figure.tight_layout()
    figure.savefig(
        output_figure,
        dpi=160,
    )

    plt.close(figure)

    final_coverage = float(
        coverage_history[-1]
    )

    print()
    print("===== PATH-AWARE RESULT =====")
    print(
        f"Selected poses: "
        f"{len(selected_indices)}"
    )
    print(
        f"Final coverage: "
        f"{final_coverage * 100.0:.2f}%"
    )
    print(
        f"Total path length: "
        f"{total_route_distance:.2f} m"
    )
    print(
        f"Total commanded rotation: "
        f"{math.degrees(total_rotation):.1f} deg"
    )

    if baseline_total_distance is not None:
        reduction = (
            1.0
            - total_route_distance
            / baseline_total_distance
        ) * 100.0

        print()
        print("===== BASELINE COMPARISON =====")
        print(
            f"Coverage-only route length: "
            f"{baseline_total_distance:.2f} m"
        )
        print(
            f"Path-aware route length: "
            f"{total_route_distance:.2f} m"
        )
        print(
            f"Route-length reduction: "
            f"{reduction:.2f}%"
        )

    print()
    print(f"Saved data: {output_data}")
    print(f"Saved figure: {output_figure}")


if __name__ == "__main__":
    main()