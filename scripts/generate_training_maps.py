#!/usr/bin/env python3

import argparse
import csv
import random
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml


def run_command(command):
    """
    Run one preprocessing command.

    Detailed output is hidden to keep the terminal readable.
    If a command fails, subprocess raises an exception immediately.
    """
    subprocess.run(
        command,
        check=True,
        stdout=subprocess.DEVNULL,
    )


def inspect_generated_map(map_directory, map_name):
    yaml_path = (
        map_directory / f"{map_name}.yaml"
    )

    pgm_path = (
        map_directory / f"{map_name}.pgm"
    )

    candidate_path = (
        map_directory / "candidate_viewpoints.npz"
    )

    with yaml_path.open("r", encoding="utf-8") as file:
        metadata = yaml.safe_load(file)

    image = cv2.imread(
        str(pgm_path),
        cv2.IMREAD_GRAYSCALE,
    )

    if image is None:
        raise RuntimeError(
            f"Cannot read generated map: {pgm_path}"
        )

    candidate_data = np.load(candidate_path)

    visibility_masks = candidate_data[
        "visibility_masks"
    ].astype(bool)

    free_mask = image >= 250

    observable_mask = np.any(
        visibility_masks,
        axis=0,
    )

    free_pixels = int(
        np.count_nonzero(free_mask)
    )

    observable_pixels = int(
        np.count_nonzero(
            observable_mask & free_mask
        )
    )

    observable_ratio = (
        observable_pixels / free_pixels
    )

    height_px, width_px = image.shape
    resolution = float(metadata["resolution"])

    return {
        "width_px": width_px,
        "height_px": height_px,
        "width_m": width_px * resolution,
        "height_m": height_px * resolution,
        "candidate_poses": len(
            candidate_data["rows"]
        ),
        "observable_ratio": observable_ratio,
    }


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Generate a varied collection of synthetic "
            "maze training maps"
        )
    )

    parser.add_argument(
        "--count",
        type=int,
        default=5,
        help="Number of maps to generate",
    )

    parser.add_argument(
        "--base-seed",
        type=int,
        default=1000,
    )

    args = parser.parse_args()

    if args.count < 1:
        raise ValueError("count must be positive")

    script_directory = (
        Path(__file__).resolve().parent
    )

    project_directory = (
        script_directory.parent
    )

    output_directory = (
        project_directory
        / "outputs"
        / "training_maps"
    )

    output_directory.mkdir(
        parents=True,
        exist_ok=True,
    )

    maze_script = (
        script_directory / "generate_maze.py"
    )

    safety_script = (
        script_directory
        / "build_navigation_mask.py"
    )

    viewpoint_script = (
        script_directory
        / "generate_viewpoints.py"
    )

    random_generator = random.Random(
        args.base_seed
    )

    summary_rows = []

    print("===== TRAINING MAP GENERATION =====")
    print(f"Python: {sys.executable}")
    print(f"Number of maps: {args.count}")
    print()

    for map_index in range(args.count):
        seed = args.base_seed + map_index

        # Randomise map geometry so that the learned policy does
        # not depend on one fixed exam-arena size.
        logical_rows = random_generator.randint(
            5, 9
        )

        logical_cols = random_generator.randint(
            6, 11
        )

        corridor_width = random_generator.choice([
            0.70,
            0.75,
            0.80,
            0.85,
            0.90,
        ])

        loop_probability = random_generator.uniform(
            0.05,
            0.25,
        )

        map_name = (
            f"train_maze_{map_index:03d}"
        )

        temporary_map_directory = (
            project_directory
            / "outputs"
            / map_name
        )

        final_map_directory = (
            output_directory / map_name
        )

        print(
            f"[{map_index + 1:02d}/{args.count:02d}] "
            f"{map_name}: "
            f"{logical_rows}x{logical_cols} cells, "
            f"corridor={corridor_width:.2f} m, "
            f"loop={loop_probability:.3f}"
        )

        # ------------------------------------------------------
        # 1. Generate occupancy map
        # ------------------------------------------------------
        run_command([
            sys.executable,
            str(maze_script),
            "--rows",
            str(logical_rows),
            "--cols",
            str(logical_cols),
            "--resolution",
            "0.05",
            "--corridor-width",
            str(corridor_width),
            "--wall-width",
            "0.10",
            "--loop-probability",
            str(loop_probability),
            "--seed",
            str(seed),
            "--name",
            map_name,
        ])

        generated_yaml = (
            temporary_map_directory
            / f"{map_name}.yaml"
        )

        # ------------------------------------------------------
        # 2. Generate safety mask
        # ------------------------------------------------------
        run_command([
            sys.executable,
            str(safety_script),
            "--map-yaml",
            str(generated_yaml),
            "--robot-radius",
            "0.12",
            "--safety-margin",
            "0.08",
        ])

        # ------------------------------------------------------
        # 3. Generate candidate viewpoints and visibility masks
        # ------------------------------------------------------
        run_command([
            sys.executable,
            str(viewpoint_script),
            "--map-yaml",
            str(generated_yaml),
            "--spacing",
            "0.75",
            "--camera-fov-deg",
            "70.0",
            "--camera-range",
            "3.0",
        ])

        # Place all training maps under outputs/training_maps.
        if final_map_directory.exists():
            raise FileExistsError(
                f"Output already exists: "
                f"{final_map_directory}"
            )

        temporary_map_directory.rename(
            final_map_directory
        )

        map_yaml = (
            final_map_directory
            / f"{map_name}.yaml"
        )

        # The YAML image path only contains the PGM filename,
        # so moving the whole directory does not break it.
        information = inspect_generated_map(
            final_map_directory,
            map_name,
        )

        summary_rows.append({
            "map_name": map_name,
            "seed": seed,
            "logical_rows": logical_rows,
            "logical_cols": logical_cols,
            "corridor_width_m": corridor_width,
            "loop_probability": loop_probability,
            **information,
        })

        print(
            f"    size="
            f"{information['width_m']:.2f}"
            f"x"
            f"{information['height_m']:.2f} m, "
            f"poses="
            f"{information['candidate_poses']}, "
            f"observable="
            f"{information['observable_ratio'] * 100.0:.2f}%"
        )

    summary_path = (
        output_directory / "map_summary.csv"
    )

    fieldnames = [
        "map_name",
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

    with summary_path.open(
        "w",
        newline="",
        encoding="utf-8",
    ) as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=fieldnames,
        )

        writer.writeheader()
        writer.writerows(summary_rows)

    observable_values = [
        row["observable_ratio"]
        for row in summary_rows
    ]

    print()
    print("===== GENERATION COMPLETE =====")
    print(
        f"Maps generated: {len(summary_rows)}"
    )
    print(
        f"Minimum observability: "
        f"{min(observable_values) * 100.0:.2f}%"
    )
    print(
        f"Mean observability: "
        f"{np.mean(observable_values) * 100.0:.2f}%"
    )
    print(f"Summary: {summary_path}")


if __name__ == "__main__":
    main()