#!/usr/bin/env python3

import argparse
import csv
import json
import random
import re
import shutil
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml


def run_command(command):
    """Run one preprocessing command and stop immediately on failure."""
    subprocess.run(
        command,
        check=True,
        stdout=subprocess.DEVNULL,
    )


def inspect_generated_map(map_directory, map_name):
    yaml_path = map_directory / f"{map_name}.yaml"
    pgm_path = map_directory / f"{map_name}.pgm"
    candidate_path = map_directory / "candidate_viewpoints.npz"

    with yaml_path.open("r", encoding="utf-8") as file:
        metadata = yaml.safe_load(file)

    image = cv2.imread(str(pgm_path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise RuntimeError(f"Cannot read generated map: {pgm_path}")

    with np.load(candidate_path, allow_pickle=False) as candidate_data:
        visibility_masks = candidate_data["visibility_masks"].astype(bool)
        candidate_poses = int(candidate_data["rows"].shape[0])

    free_mask = image >= 250
    observable_mask = np.any(visibility_masks, axis=0)
    free_pixels = int(np.count_nonzero(free_mask))
    observable_pixels = int(
        np.count_nonzero(observable_mask & free_mask)
    )

    if free_pixels == 0:
        raise RuntimeError(f"Generated map has no free pixels: {map_name}")

    height_px, width_px = image.shape
    resolution = float(metadata["resolution"])

    return {
        "width_px": width_px,
        "height_px": height_px,
        "width_m": width_px * resolution,
        "height_m": height_px * resolution,
        "candidate_poses": candidate_poses,
        "observable_ratio": observable_pixels / free_pixels,
    }


def parse_arguments():
    parser = argparse.ArgumentParser(
        description=(
            "Generate a collision-safe, reproducible collection "
            "of synthetic maze training maps"
        )
    )
    parser.add_argument(
        "--count",
        type=int,
        default=30,
        help="Number of maps to generate",
    )
    parser.add_argument(
        "--base-seed",
        type=int,
        default=13033,
        help="Seed used for geometry sampling and the first maze",
    )
    parser.add_argument(
        "--name-prefix",
        default="general_maze",
        help="Filename and directory prefix for generated maps",
    )
    parser.add_argument(
        "--start-index",
        type=int,
        default=0,
        help="First numeric map index",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=None,
        help=(
            "Independent output directory. The default is "
            "<project>/outputs/training_maps_v2"
        ),
    )
    return parser.parse_args()


def validate_arguments(args):
    if args.count < 1:
        raise ValueError("count must be positive")
    if args.start_index < 0:
        raise ValueError("start-index must be non-negative")
    if not re.fullmatch(r"[A-Za-z0-9_-]+", args.name_prefix):
        raise ValueError(
            "name-prefix may contain only letters, digits, '_' and '-'"
        )


def preflight(
    output_directory,
    temporary_root,
    map_names,
    required_scripts,
):
    missing_scripts = [
        str(path) for path in required_scripts if not path.is_file()
    ]
    if missing_scripts:
        raise FileNotFoundError(
            "Required preprocessing scripts are missing: "
            + ", ".join(missing_scripts)
        )

    if output_directory.exists() and any(output_directory.iterdir()):
        raise FileExistsError(
            "Output directory already exists and is not empty: "
            f"{output_directory}\n"
            "Choose another --output-root. Nothing was overwritten."
        )

    conflicts = []
    for map_name in map_names:
        temporary_path = temporary_root / map_name
        final_path = output_directory / map_name
        if temporary_path.exists():
            conflicts.append(str(temporary_path))
        if final_path.exists():
            conflicts.append(str(final_path))

    if conflicts:
        raise FileExistsError(
            "Preflight found existing map directories:\n  "
            + "\n  ".join(conflicts)
            + "\nNothing was generated or deleted."
        )


def main():
    args = parse_arguments()
    validate_arguments(args)

    script_directory = Path(__file__).resolve().parent
    project_directory = script_directory.parent
    temporary_root = project_directory / "outputs"

    if args.output_root is None:
        output_directory = temporary_root / "training_maps_v2"
    else:
        output_directory = args.output_root.expanduser().resolve()

    maze_script = script_directory / "generate_maze.py"
    safety_script = script_directory / "build_navigation_mask.py"
    viewpoint_script = script_directory / "generate_viewpoints.py"

    map_names = [
        f"{args.name_prefix}_{index:03d}"
        for index in range(
            args.start_index,
            args.start_index + args.count,
        )
    ]

    preflight(
        output_directory,
        temporary_root,
        map_names,
        (maze_script, safety_script, viewpoint_script),
    )
    output_directory.mkdir(parents=True, exist_ok=True)

    random_generator = random.Random(args.base_seed)
    summary_rows = []

    configuration = {
        "count": args.count,
        "base_seed": args.base_seed,
        "name_prefix": args.name_prefix,
        "start_index": args.start_index,
        "resolution_m_per_pixel": 0.05,
        "wall_width_m": 0.10,
        "robot_radius_m": 0.12,
        "safety_margin_m": 0.08,
        "candidate_spacing_m": 0.75,
        "camera_fov_deg": 70.0,
        "camera_range_m": 3.0,
        "logical_rows_range": [5, 9],
        "logical_cols_range": [6, 11],
        "corridor_width_choices_m": [0.70, 0.75, 0.80, 0.85, 0.90],
        "loop_probability_range": [0.05, 0.25],
    }

    print("===== TRAINING MAP GENERATION V2 =====")
    print(f"Python: {sys.executable}")
    print(f"Number of maps: {args.count}")
    print(f"Base seed: {args.base_seed}")
    print(f"Name prefix: {args.name_prefix}")
    print(f"Output root: {output_directory}")
    print()

    for local_index, map_name in enumerate(map_names):
        map_index = args.start_index + local_index
        seed = args.base_seed + local_index
        logical_rows = random_generator.randint(5, 9)
        logical_cols = random_generator.randint(6, 11)
        corridor_width = random_generator.choice(
            [0.70, 0.75, 0.80, 0.85, 0.90]
        )
        loop_probability = random_generator.uniform(0.05, 0.25)

        temporary_map_directory = temporary_root / map_name
        final_map_directory = output_directory / map_name

        print(
            f"[{local_index + 1:02d}/{args.count:02d}] "
            f"{map_name}: {logical_rows}x{logical_cols} cells, "
            f"corridor={corridor_width:.2f} m, "
            f"loop={loop_probability:.3f}"
        )

        try:
            run_command([
                sys.executable,
                str(maze_script),
                "--rows", str(logical_rows),
                "--cols", str(logical_cols),
                "--resolution", "0.05",
                "--corridor-width", str(corridor_width),
                "--wall-width", "0.10",
                "--loop-probability", str(loop_probability),
                "--seed", str(seed),
                "--name", map_name,
            ])

            generated_yaml = temporary_map_directory / f"{map_name}.yaml"

            run_command([
                sys.executable,
                str(safety_script),
                "--map-yaml", str(generated_yaml),
                "--robot-radius", "0.12",
                "--safety-margin", "0.08",
            ])

            run_command([
                sys.executable,
                str(viewpoint_script),
                "--map-yaml", str(generated_yaml),
                "--spacing", "0.75",
                "--camera-fov-deg", "70.0",
                "--camera-range", "3.0",
            ])

            if not temporary_map_directory.is_dir():
                raise RuntimeError(
                    f"Expected temporary output was not created: "
                    f"{temporary_map_directory}"
                )
            if final_map_directory.exists():
                raise FileExistsError(
                    f"Final output unexpectedly exists: "
                    f"{final_map_directory}"
                )

            shutil.move(
                str(temporary_map_directory),
                str(final_map_directory),
            )

            information = inspect_generated_map(
                final_map_directory,
                map_name,
            )
        except Exception:
            print()
            print("===== GENERATION STOPPED =====")
            print(f"Failed map: {map_name}")
            print(
                "No automatic deletion was performed. Inspect these paths:"
            )
            print(f"  Temporary: {temporary_map_directory}")
            print(f"  Final:     {final_map_directory}")
            raise

        summary_rows.append({
            "map_name": map_name,
            "map_index": map_index,
            "seed": seed,
            "logical_rows": logical_rows,
            "logical_cols": logical_cols,
            "corridor_width_m": corridor_width,
            "loop_probability": loop_probability,
            **information,
        })

        print(
            f"    size={information['width_m']:.2f}"
            f"x{information['height_m']:.2f} m, "
            f"poses={information['candidate_poses']}, "
            f"observable={information['observable_ratio'] * 100.0:.2f}%"
        )

    summary_path = output_directory / "map_summary.csv"
    fieldnames = [
        "map_name",
        "map_index",
        "seed",
        "logical_rows",
        "logical_cols",
        "corridor_width_m",
        "loop_probability",
        "width_px",
        "height_px",
        "width_m",
        "height_m",
        "candidate_poses",
        "observable_ratio",
    ]

    with summary_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(summary_rows)

    config_path = output_directory / "generation_config.json"
    with config_path.open("w", encoding="utf-8") as file:
        json.dump(configuration, file, indent=2)
        file.write("\n")

    observable_values = [
        row["observable_ratio"] for row in summary_rows
    ]

    print()
    print("===== GENERATION COMPLETE =====")
    print(f"Maps generated: {len(summary_rows)}")
    print(
        "Minimum observability: "
        f"{min(observable_values) * 100.0:.2f}%"
    )
    print(
        "Mean observability: "
        f"{np.mean(observable_values) * 100.0:.2f}%"
    )
    print(f"Summary: {summary_path}")
    print(f"Configuration: {config_path}")


if __name__ == "__main__":
    main()
