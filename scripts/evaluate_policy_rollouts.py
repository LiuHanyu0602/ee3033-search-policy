#!/usr/bin/env python3
import argparse
import csv
import math
from pathlib import Path

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from generate_expert_dataset import (
    build_pairwise_paths,
    candidate_features,
    expert_scores,
    make_shortlist,
    wrap_angle,
)


STRATEGIES = (
    "coverage_only",
    "immediate_time_aware",
    "two_step_expert",
    "tiny_mlp",
)

DISPLAY_NAMES = {
    "coverage_only": "Coverage only",
    "immediate_time_aware": "Immediate time-aware",
    "two_step_expert": "Two-step expert",
    "tiny_mlp": "Tiny MLP",
}


def load_map(map_dir):
    with np.load(
        map_dir / "candidate_viewpoints.npz",
        allow_pickle=False,
    ) as data:
        result = {
            "rows": data["rows"].astype(np.int32),
            "cols": data["cols"].astype(np.int32),
            "world_x": data["world_x"].astype(np.float64),
            "world_y": data["world_y"].astype(np.float64),
            "headings": data["headings"].astype(np.float64),
            "visible_areas": data["visible_areas_m2"].astype(np.float64),
            "visibility": data["visibility_masks"].astype(bool),
            "resolution": float(data["resolution"]),
        }

    nav_image = cv2.imread(
        str(map_dir / "navigation_mask.png"),
        cv2.IMREAD_GRAYSCALE,
    )
    if nav_image is None:
        raise RuntimeError("Could not load navigation_mask.png")

    result["nav_mask"] = nav_image >= 128
    result["clearance"] = np.load(
        map_dir / "clearance_metres.npy",
        allow_pickle=False,
    )
    result["observable"] = np.any(result["visibility"], axis=0)

    print("Computing all-pairs candidate path distances...")
    result["paths"] = build_pairwise_paths(
        result["nav_mask"],
        result["rows"],
        result["cols"],
        result["resolution"],
    )
    return result


def load_numpy_model(path):
    with np.load(path, allow_pickle=False) as data:
        return {key: data[key].copy() for key in data.files}


def numpy_mlp(features, weights):
    normalized = (
        features - weights["feature_mean"]
    ) / weights["feature_std"]
    hidden_1 = np.maximum(
        normalized @ weights["weight_1"].T + weights["bias_1"],
        0.0,
    )
    hidden_2 = np.maximum(
        hidden_1 @ weights["weight_2"].T + weights["bias_2"],
        0.0,
    )
    return (
        hidden_2 @ weights["weight_3"].T + weights["bias_3"]
    ).squeeze(-1)


def action_times(current_pose, map_data, args):
    distances = map_data["paths"][current_pose].astype(np.float64)
    turns = np.abs(
        wrap_angle(
            map_data["headings"] - map_data["headings"][current_pose]
        )
    )
    times = (
        distances / args.linear_speed
        + turns / args.angular_speed
        + args.observation_time
    )
    return distances, turns, times


def select_action(
    strategy,
    current_pose,
    searched,
    visits,
    map_data,
    weights,
    args,
):
    visibility = map_data["visibility"]
    unseen = map_data["observable"] & ~searched
    unseen_count = int(np.count_nonzero(unseen))
    gain_counts = np.count_nonzero(
        visibility & unseen[None, :, :],
        axis=(1, 2),
    )
    distances, turns, times = action_times(current_pose, map_data, args)
    valid = np.isfinite(distances) & (gain_counts > 0)

    if not np.any(valid):
        return None

    if strategy == "coverage_only":
        values = gain_counts.astype(np.float64)
        values[~valid] = -np.inf
        return int(np.argmax(values))

    if strategy == "immediate_time_aware":
        values = (
            gain_counts / max(unseen_count, 1)
        ) * np.exp(-times / args.time_discount)
        values[~valid] = -np.inf
        return int(np.argmax(values))

    shortlist, current_times = make_shortlist(
        current_pose,
        gain_counts,
        map_data["paths"],
        map_data["headings"],
        args.linear_speed,
        args.angular_speed,
        args.observation_time,
        args.time_discount,
        args.shortlist_size,
    )
    if shortlist.size == 0:
        return None

    if strategy == "two_step_expert":
        values = expert_scores(
            shortlist,
            current_times,
            unseen,
            visibility,
            map_data["paths"],
            map_data["headings"],
            args.linear_speed,
            args.angular_speed,
            args.observation_time,
            args.time_discount,
            args.successor_count,
        )
        return int(shortlist[int(np.argmax(values))])

    if strategy == "tiny_mlp":
        features = candidate_features(
            shortlist,
            current_pose,
            gain_counts,
            unseen_count,
            searched,
            visibility,
            map_data["visible_areas"],
            map_data["paths"],
            map_data["headings"],
            map_data["clearance"],
            map_data["rows"],
            map_data["cols"],
            map_data["world_x"],
            map_data["world_y"],
            visits,
            map_data["resolution"],
            args.linear_speed,
            args.angular_speed,
            args.observation_time,
        )
        values = numpy_mlp(features, weights)
        return int(shortlist[int(np.argmax(values))])

    raise ValueError(f"Unknown strategy: {strategy}")


def rollout(strategy, start_pose, map_data, weights, args):
    visibility = map_data["visibility"]
    observable = map_data["observable"]
    observable_count = int(np.count_nonzero(observable))

    current_pose = int(start_pose)
    searched = visibility[current_pose].copy() & observable
    visits = np.zeros(visibility.shape[0], dtype=np.int32)
    visits[current_pose] += 1

    elapsed_time = args.observation_time
    path_length = 0.0
    rotation = 0.0
    first_seen_time = np.full(observable.shape, np.inf, dtype=np.float64)
    first_seen_time[searched] = elapsed_time

    coverage_history = [
        np.count_nonzero(searched) / max(observable_count, 1)
    ]
    time_history = [elapsed_time]

    for step in range(args.max_steps):
        coverage = np.count_nonzero(
            searched & observable
        ) / max(observable_count, 1)
        if coverage >= args.coverage_target:
            break

        next_pose = select_action(
            strategy,
            current_pose,
            searched,
            visits,
            map_data,
            weights,
            args,
        )
        if next_pose is None:
            break

        distances, turns, times = action_times(
            current_pose,
            map_data,
            args,
        )
        path_length += float(distances[next_pose])
        rotation += float(turns[next_pose])
        elapsed_time += float(times[next_pose])

        newly_seen = visibility[next_pose] & observable & ~searched
        first_seen_time[newly_seen] = elapsed_time
        searched |= visibility[next_pose]
        visits[next_pose] += 1
        current_pose = next_pose

        coverage_history.append(
            np.count_nonzero(searched & observable)
            / max(observable_count, 1)
        )
        time_history.append(elapsed_time)

    final_coverage = np.count_nonzero(
        searched & observable
    ) / max(observable_count, 1)

    # Uniform target-location assumption. Unseen target pixels receive a
    # fixed censoring penalty so incomplete policies are not rewarded.
    censor_time = elapsed_time + args.undetected_penalty
    target_times = np.where(
        np.isfinite(first_seen_time[observable]),
        first_seen_time[observable],
        censor_time,
    )

    return {
        "strategy": strategy,
        "start_pose": int(start_pose),
        "steps": len(time_history) - 1,
        "coverage": final_coverage,
        "total_time_s": elapsed_time,
        "expected_detection_time_s": float(np.mean(target_times)),
        "path_length_m": path_length,
        "rotation_rad": rotation,
        "time_history": np.asarray(time_history),
        "coverage_history": np.asarray(coverage_history),
    }


def mean_std(values):
    values = np.asarray(values, dtype=np.float64)
    return float(np.mean(values)), float(np.std(values, ddof=0))


def save_csv(results, path):
    fields = [
        "map_name",
        "strategy",
        "start_pose",
        "steps",
        "coverage",
        "total_time_s",
        "expected_detection_time_s",
        "path_length_m",
        "rotation_rad",
    ]
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        for result in results:
            writer.writerow({key: result[key] for key in fields})


def save_figure(results, path, map_count, episodes_per_map):
    metrics = (
        ("expected_detection_time_s", "Expected detection time (s)"),
        ("total_time_s", "Time to stopping condition (s)"),
        ("path_length_m", "Path length (m)"),
        ("rotation_rad", "Commanded rotation (rad)"),
    )
    colors = ("#7f8c8d", "#3498db", "#e67e22", "#2ecc71")
    figure, axes = plt.subplots(2, 2, figsize=(11, 7.5))

    for axis, (metric, label) in zip(axes.flat, metrics):
        means = []
        stds = []
        for strategy in STRATEGIES:
            values = [
                result[metric]
                for result in results
                if result["strategy"] == strategy
            ]
            mean, std = mean_std(values)
            means.append(mean)
            stds.append(std)

        positions = np.arange(len(STRATEGIES))
        axis.bar(positions, means, yerr=stds, capsize=4, color=colors)
        axis.set_xticks(positions)
        axis.set_xticklabels(
            [DISPLAY_NAMES[item] for item in STRATEGIES],
            rotation=18,
            ha="right",
        )
        axis.set_ylabel(label)
        axis.grid(axis="y", alpha=0.25)

    figure.suptitle(
        f"Held-out policy rollouts: {map_count} maps, "
        f"{episodes_per_map} starts per map"
    )
    figure.tight_layout()
    figure.savefig(path, dpi=180)
    plt.close(figure)


def print_aggregate(results, heading):
    print(f"\n===== {heading} =====")
    for strategy in STRATEGIES:
        strategy_results = [
            result for result in results
            if result["strategy"] == strategy
        ]
        coverage_mean, coverage_std = mean_std([
            item["coverage"] * 100 for item in strategy_results
        ])
        detection_mean, detection_std = mean_std([
            item["expected_detection_time_s"]
            for item in strategy_results
        ])
        time_mean, time_std = mean_std([
            item["total_time_s"] for item in strategy_results
        ])
        path_mean, path_std = mean_std([
            item["path_length_m"] for item in strategy_results
        ])
        rotation_mean, rotation_std = mean_std([
            math.degrees(item["rotation_rad"])
            for item in strategy_results
        ])

        print(DISPLAY_NAMES[strategy])
        print(
            f"  Coverage: {coverage_mean:.2f} +/- "
            f"{coverage_std:.2f}%"
        )
        print(
            f"  Expected detection: {detection_mean:.2f} +/- "
            f"{detection_std:.2f} s"
        )
        print(
            f"  Total time: {time_mean:.2f} +/- "
            f"{time_std:.2f} s"
        )
        print(
            f"  Path length: {path_mean:.2f} +/- "
            f"{path_std:.2f} m"
        )
        print(
            f"  Rotation: {rotation_mean:.1f} +/- "
            f"{rotation_std:.1f} deg"
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--training-root",
        type=Path,
        default=Path(
            "~/ee3033_ws/src/ee3033_search_policy/"
            "outputs/training_maps_v2"
        ).expanduser(),
    )
    parser.add_argument(
        "--map-names",
        nargs="+",
        default=[
            "general_maze_023",
            "general_maze_016",
            "general_maze_014",
            "general_maze_012",
            "general_maze_008",
            "general_maze_026",
        ],
        help="Held-out map directory names to evaluate",
    )
    parser.add_argument(
        "--episodes",
        type=int,
        default=3,
        help="Shared random start poses per map",
    )
    parser.add_argument("--max-steps", type=int, default=100)
    parser.add_argument("--coverage-target", type=float, default=0.995)
    parser.add_argument("--shortlist-size", type=int, default=32)
    parser.add_argument("--successor-count", type=int, default=16)
    parser.add_argument("--linear-speed", type=float, default=0.25)
    parser.add_argument("--angular-speed", type=float, default=0.8)
    parser.add_argument("--observation-time", type=float, default=0.25)
    parser.add_argument("--time-discount", type=float, default=20.0)
    parser.add_argument("--undetected-penalty", type=float, default=60.0)
    parser.add_argument("--seed", type=int, default=44033)
    args = parser.parse_args()

    if args.episodes < 1:
        raise ValueError("episodes must be positive")
    if len(set(args.map_names)) != len(args.map_names):
        raise ValueError("map-names contains duplicates")

    weights_path = (
        args.training_root
        / "tiny_mlp"
        / "tiny_search_policy_weights.npz"
    )
    output_dir = (
        args.training_root
        / "policy_evaluation"
        / "held_out_test_maps"
    )
    output_dir.mkdir(parents=True, exist_ok=True)

    weights = load_numpy_model(weights_path)
    rng = np.random.default_rng(args.seed)

    print("===== MULTI-MAP POLICY ROLLOUT EVALUATION =====")
    print("Held-out maps:", args.map_names)
    print("Episodes per map:", args.episodes)
    print("Total paired trials:", len(args.map_names) * args.episodes)

    results = []
    for map_index, map_name in enumerate(args.map_names, start=1):
        map_dir = args.training_root / map_name
        if not map_dir.is_dir():
            raise FileNotFoundError(f"Map directory not found: {map_dir}")

        print(
            f"\n===== MAP {map_index}/{len(args.map_names)}: "
            f"{map_name} ====="
        )
        map_data = load_map(map_dir)
        pose_count = map_data["visibility"].shape[0]
        if args.episodes > pose_count:
            raise ValueError(
                f"episodes={args.episodes} exceeds {pose_count} poses "
                f"on {map_name}"
            )
        start_poses = rng.choice(
            pose_count,
            size=args.episodes,
            replace=False,
        )
        print("Shared start poses:", start_poses.tolist())

        map_results = []
        for episode, start_pose in enumerate(start_poses, start=1):
            print(
                f"\nEpisode {episode}/{args.episodes}, "
                f"start pose {start_pose}"
            )
            for strategy in STRATEGIES:
                result = rollout(
                    strategy,
                    int(start_pose),
                    map_data,
                    weights,
                    args,
                )
                result["map_name"] = map_name
                map_results.append(result)
                results.append(result)
                print(
                    f"  {DISPLAY_NAMES[strategy]:22s} | "
                    f"coverage={result['coverage'] * 100:6.2f}% | "
                    f"steps={result['steps']:3d} | "
                    f"E[T]={result['expected_detection_time_s']:7.2f}s | "
                    f"path={result['path_length_m']:7.2f}m"
                )

        print_aggregate(map_results, f"{map_name} AGGREGATE")

    print_aggregate(results, "GLOBAL AGGREGATE RESULTS")

    csv_path = output_dir / "rollout_metrics.csv"
    figure_path = output_dir / "policy_comparison.png"
    save_csv(results, csv_path)
    save_figure(
        results,
        figure_path,
        len(args.map_names),
        args.episodes,
    )

    print("\nSaved metrics:", csv_path)
    print("Saved figure:", figure_path)


if __name__ == "__main__":
    main()
