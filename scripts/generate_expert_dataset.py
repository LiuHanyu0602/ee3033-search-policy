#!/usr/bin/env python3
import argparse
import heapq
import math
from pathlib import Path

import cv2
import numpy as np


MOVES = (
    (-1, 0, 1.0),
    (1, 0, 1.0),
    (0, -1, 1.0),
    (0, 1, 1.0),
    (-1, -1, math.sqrt(2.0)),
    (-1, 1, math.sqrt(2.0)),
    (1, -1, math.sqrt(2.0)),
    (1, 1, math.sqrt(2.0)),
)

FEATURE_NAMES = np.asarray([
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
])


def wrap_angle(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def dijkstra(nav_mask, start_row, start_col, resolution):
    height, width = nav_mask.shape
    distances = np.full((height, width), np.inf, dtype=np.float64)

    if not nav_mask[start_row, start_col]:
        return distances

    distances[start_row, start_col] = 0.0
    queue = [(0.0, start_row, start_col)]

    while queue:
        current_distance, row, col = heapq.heappop(queue)

        if current_distance > distances[row, col] + 1e-12:
            continue

        for dr, dc, pixel_cost in MOVES:
            new_row = row + dr
            new_col = col + dc

            if not (0 <= new_row < height and 0 <= new_col < width):
                continue
            if not nav_mask[new_row, new_col]:
                continue

            # Prevent a diagonal move from cutting through a wall corner.
            if dr != 0 and dc != 0:
                if not nav_mask[row + dr, col]:
                    continue
                if not nav_mask[row, col + dc]:
                    continue

            candidate_distance = (
                current_distance + pixel_cost * resolution
            )

            if candidate_distance + 1e-12 < distances[new_row, new_col]:
                distances[new_row, new_col] = candidate_distance
                heapq.heappush(
                    queue,
                    (candidate_distance, new_row, new_col),
                )

    return distances


def build_pairwise_paths(nav_mask, rows, cols, resolution):
    position_to_index = {}
    unique_positions = []
    pose_position_indices = []

    for row, col in zip(rows, cols):
        position = (int(row), int(col))

        if position not in position_to_index:
            position_to_index[position] = len(unique_positions)
            unique_positions.append(position)

        pose_position_indices.append(position_to_index[position])

    pose_position_indices = np.asarray(
        pose_position_indices,
        dtype=np.int32,
    )
    position_count = len(unique_positions)
    position_paths = np.full(
        (position_count, position_count),
        np.inf,
        dtype=np.float32,
    )

    for source_index, (row, col) in enumerate(unique_positions):
        distance_field = dijkstra(
            nav_mask,
            row,
            col,
            resolution,
        )

        for destination_index, (dst_row, dst_col) in enumerate(
            unique_positions
        ):
            position_paths[source_index, destination_index] = (
                distance_field[dst_row, dst_col]
            )

    return position_paths[
        pose_position_indices[:, None],
        pose_position_indices[None, :],
    ]


def candidate_features(
    indices,
    current_pose,
    gain_counts,
    unseen_count,
    searched,
    visibility_masks,
    visible_areas,
    pairwise_paths,
    headings,
    clearance,
    rows,
    cols,
    world_x,
    world_y,
    visits,
    resolution,
    linear_speed,
    angular_speed,
    observation_time,
):
    indices = np.asarray(indices, dtype=np.int32)
    distances = pairwise_paths[current_pose, indices].astype(np.float64)
    turns = np.abs(
        wrap_angle(headings[indices] - headings[current_pose])
    )
    times = (
        distances / linear_speed
        + turns / angular_speed
        + observation_time
    )

    gain_areas = gain_counts[indices] * resolution**2
    gain_fractions = gain_counts[indices] / max(unseen_count, 1)
    candidate_clearance = clearance[rows[indices], cols[indices]]
    searched_count = int(np.count_nonzero(searched))
    observable_count = searched_count + unseen_count
    coverage = searched_count / max(observable_count, 1)

    visible_counts = np.count_nonzero(
        visibility_masks[indices],
        axis=(1, 2),
    )
    redundancy = (
        visible_counts - gain_counts[indices]
    ) / np.maximum(visible_counts, 1)

    dx = world_x[indices] - world_x[current_pose]
    dy = world_y[indices] - world_y[current_pose]
    euclidean = np.hypot(dx, dy)
    detour = distances / np.maximum(euclidean, resolution)
    detour[euclidean < 0.5 * resolution] = 1.0

    return np.column_stack([
        gain_areas,
        gain_fractions,
        distances,
        turns,
        times,
        candidate_clearance,
        visits[indices],
        np.full(indices.size, coverage),
        visible_areas[indices],
        redundancy,
        euclidean,
        detour,
    ]).astype(np.float32)


def make_shortlist(
    current_pose,
    gain_counts,
    pairwise_paths,
    headings,
    linear_speed,
    angular_speed,
    observation_time,
    time_discount,
    shortlist_size,
):
    distances = pairwise_paths[current_pose].astype(np.float64)
    turns = np.abs(wrap_angle(headings - headings[current_pose]))
    times = (
        distances / linear_speed
        + turns / angular_speed
        + observation_time
    )
    reachable = np.isfinite(distances) & (gain_counts > 0)

    if not np.any(reachable):
        return np.empty(0, dtype=np.int32), times

    immediate = gain_counts * np.exp(-times / time_discount)
    immediate[~reachable] = -np.inf

    shortlist_size = min(shortlist_size, int(np.count_nonzero(reachable)))
    shortlist = np.argpartition(
        immediate,
        -shortlist_size,
    )[-shortlist_size:]
    shortlist = shortlist[np.argsort(immediate[shortlist])[::-1]]

    return shortlist.astype(np.int32), times


def expert_scores(
    shortlist,
    current_times,
    unseen,
    visibility_masks,
    pairwise_paths,
    headings,
    linear_speed,
    angular_speed,
    observation_time,
    time_discount,
    successor_count,
):
    unseen_count = int(np.count_nonzero(unseen))
    scores = np.zeros(shortlist.size, dtype=np.float64)

    for output_index, first_pose in enumerate(shortlist):
        first_new = visibility_masks[first_pose] & unseen
        first_gain = int(np.count_nonzero(first_new))
        first_time = float(current_times[first_pose])

        first_value = (
            first_gain / max(unseen_count, 1)
        ) * math.exp(-first_time / time_discount)

        remaining = unseen & ~visibility_masks[first_pose]
        second_gains = np.count_nonzero(
            visibility_masks & remaining[None, :, :],
            axis=(1, 2),
        )

        second_distances = pairwise_paths[first_pose].astype(np.float64)
        second_turns = np.abs(
            wrap_angle(headings - headings[first_pose])
        )
        second_times = (
            second_distances / linear_speed
            + second_turns / angular_speed
            + observation_time
        )
        valid = np.isfinite(second_distances) & (second_gains > 0)

        second_value = 0.0

        if np.any(valid):
            second_utility = (
                second_gains / max(unseen_count, 1)
            ) * np.exp(
                -(first_time + second_times) / time_discount
            )
            second_utility[~valid] = -np.inf

            count = min(successor_count, int(np.count_nonzero(valid)))
            best_successors = np.argpartition(
                second_utility,
                -count,
            )[-count:]
            second_value = float(np.max(second_utility[best_successors]))

        scores[output_index] = first_value + second_value

    return scores


def process_map(map_dir, args, rng, map_index, state_offset):
    data_path = map_dir / "candidate_viewpoints.npz"
    nav_path = map_dir / "navigation_mask.png"
    clearance_path = map_dir / "clearance_metres.npy"

    with np.load(data_path, allow_pickle=False) as data:
        rows = data["rows"].astype(np.int32)
        cols = data["cols"].astype(np.int32)
        world_x = data["world_x"].astype(np.float64)
        world_y = data["world_y"].astype(np.float64)
        headings = data["headings"].astype(np.float64)
        visible_areas = data["visible_areas_m2"].astype(np.float64)
        visibility_masks = data["visibility_masks"].astype(bool)
        resolution = float(data["resolution"])

    nav_image = cv2.imread(str(nav_path), cv2.IMREAD_GRAYSCALE)
    if nav_image is None:
        raise RuntimeError(f"Could not read {nav_path}")

    nav_mask = nav_image >= 128
    clearance = np.load(clearance_path, allow_pickle=False)

    if visibility_masks.shape[1:] != nav_mask.shape:
        raise ValueError(f"Shape mismatch in {map_dir.name}")
    if not np.all(nav_mask[rows, cols]):
        raise ValueError(f"Unsafe candidate found in {map_dir.name}")

    observable = np.any(visibility_masks, axis=0)
    observable_count = int(np.count_nonzero(observable))
    pairwise_paths = build_pairwise_paths(
        nav_mask,
        rows,
        cols,
        resolution,
    )

    all_x = []
    all_y = []
    all_best = []
    all_state = []
    all_candidate = []
    episode_summaries = []
    next_state_id = state_offset

    for episode in range(args.episodes_per_map):
        current_pose = int(rng.integers(0, len(rows)))
        searched = visibility_masks[current_pose].copy() & observable
        visits = np.zeros(len(rows), dtype=np.int32)
        visits[current_pose] += 1
        elapsed_time = args.observation_time

        for step in range(args.max_steps):
            unseen = observable & ~searched
            unseen_count = int(np.count_nonzero(unseen))
            coverage = 1.0 - unseen_count / max(observable_count, 1)

            if coverage >= args.coverage_target:
                break

            gain_counts = np.count_nonzero(
                visibility_masks & unseen[None, :, :],
                axis=(1, 2),
            )

            shortlist, current_times = make_shortlist(
                current_pose,
                gain_counts,
                pairwise_paths,
                headings,
                args.linear_speed,
                args.angular_speed,
                args.observation_time,
                args.time_discount,
                args.shortlist_size,
            )

            if shortlist.size == 0:
                break

            scores = expert_scores(
                shortlist,
                current_times,
                unseen,
                visibility_masks,
                pairwise_paths,
                headings,
                args.linear_speed,
                args.angular_speed,
                args.observation_time,
                args.time_discount,
                args.successor_count,
            )

            best_local_index = int(np.argmax(scores))
            best_pose = int(shortlist[best_local_index])

            score_min = float(np.min(scores))
            score_max = float(np.max(scores))
            if score_max > score_min + 1e-12:
                targets = (scores - score_min) / (score_max - score_min)
            else:
                targets = np.zeros_like(scores)

            features = candidate_features(
                shortlist,
                current_pose,
                gain_counts,
                unseen_count,
                searched,
                visibility_masks,
                visible_areas,
                pairwise_paths,
                headings,
                clearance,
                rows,
                cols,
                world_x,
                world_y,
                visits,
                resolution,
                args.linear_speed,
                args.angular_speed,
                args.observation_time,
            )

            best_flags = np.zeros(shortlist.size, dtype=np.uint8)
            best_flags[best_local_index] = 1

            all_x.append(features)
            all_y.append(targets.astype(np.float32))
            all_best.append(best_flags)
            all_state.append(
                np.full(shortlist.size, next_state_id, dtype=np.int32)
            )
            all_candidate.append(shortlist.astype(np.int32))

            elapsed_time += float(current_times[best_pose])
            searched |= visibility_masks[best_pose]
            visits[best_pose] += 1
            current_pose = best_pose
            next_state_id += 1

        final_coverage = (
            np.count_nonzero(searched & observable)
            / max(observable_count, 1)
        )
        episode_summaries.append((final_coverage, elapsed_time, step + 1))

    return {
        "x": np.concatenate(all_x, axis=0),
        "y": np.concatenate(all_y, axis=0),
        "best": np.concatenate(all_best, axis=0),
        "state": np.concatenate(all_state, axis=0),
        "candidate": np.concatenate(all_candidate, axis=0),
        "map": np.full(
            sum(array.shape[0] for array in all_x),
            map_index,
            dtype=np.int16,
        ),
        "state_count": next_state_id - state_offset,
        "summaries": episode_summaries,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--root",
        type=Path,
        default=Path(
            "~/ee3033_ws/src/ee3033_search_policy/"
            "outputs/training_maps"
        ).expanduser(),
    )
    parser.add_argument("--episodes-per-map", type=int, default=3)
    parser.add_argument("--max-steps", type=int, default=45)
    parser.add_argument("--coverage-target", type=float, default=0.995)
    parser.add_argument("--shortlist-size", type=int, default=32)
    parser.add_argument("--successor-count", type=int, default=16)
    parser.add_argument("--linear-speed", type=float, default=0.25)
    parser.add_argument("--angular-speed", type=float, default=0.8)
    parser.add_argument("--observation-time", type=float, default=0.25)
    parser.add_argument("--time-discount", type=float, default=20.0)
    parser.add_argument("--seed", type=int, default=3033)
    args = parser.parse_args()

    map_dirs = sorted(
        path for path in args.root.iterdir()
        if path.is_dir() and (path / "candidate_viewpoints.npz").exists()
    )
    if not map_dirs:
        raise RuntimeError(f"No training maps found in {args.root}")

    rng = np.random.default_rng(args.seed)
    results = []
    state_offset = 0

    print("===== EXPERT DATASET GENERATION =====")
    print("Maps:", len(map_dirs))
    print("Features:", len(FEATURE_NAMES))

    for map_index, map_dir in enumerate(map_dirs):
        print(f"[{map_index + 1:02d}/{len(map_dirs):02d}] {map_dir.name}")
        result = process_map(
            map_dir,
            args,
            rng,
            map_index,
            state_offset,
        )
        state_offset += result["state_count"]
        results.append(result)

        coverages = [item[0] for item in result["summaries"]]
        times = [item[1] for item in result["summaries"]]
        print(
            f"    states={result['state_count']}, "
            f"samples={result['x'].shape[0]}, "
            f"coverage={np.mean(coverages) * 100:.2f}%, "
            f"estimated_time={np.mean(times):.1f} s"
        )

    output_path = args.root / "expert_dataset.npz"
    np.savez_compressed(
        output_path,
        features=np.concatenate([item["x"] for item in results]),
        targets=np.concatenate([item["y"] for item in results]),
        is_best=np.concatenate([item["best"] for item in results]),
        state_ids=np.concatenate([item["state"] for item in results]),
        map_ids=np.concatenate([item["map"] for item in results]),
        candidate_ids=np.concatenate(
            [item["candidate"] for item in results]
        ),
        feature_names=FEATURE_NAMES,
        map_names=np.asarray([path.name for path in map_dirs]),
        seed=np.asarray(args.seed, dtype=np.int32),
    )

    with np.load(output_path, allow_pickle=False) as dataset:
        x = dataset["features"]
        y = dataset["targets"]
        best = dataset["is_best"]
        states = dataset["state_ids"]

    print("\n===== DATASET COMPLETE =====")
    print("Output:", output_path)
    print("Feature matrix:", x.shape)
    print("Target vector:", y.shape)
    print("Decision states:", np.unique(states).size)
    print("Positive expert choices:", int(best.sum()))
    print("Finite features:", bool(np.all(np.isfinite(x))))
    print("Target range:", float(y.min()), "to", float(y.max()))


if __name__ == "__main__":
    main()
