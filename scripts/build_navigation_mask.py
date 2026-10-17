#!/usr/bin/env python3

import argparse
from collections import deque
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
import yaml


def classify_ros_map(image):
    """
    Classify a ROS map saved by map_saver.

    0   approximately occupied
    254 approximately free
    205 approximately unknown

    Explicit thresholds prevent value 205 from being incorrectly
    treated as free space.
    """
    occupied = image <= 100
    free = image >= 250
    unknown = ~(occupied | free)

    return free, occupied, unknown


def reachable_ratio(binary_mask):
    """
    Return the fraction of true pixels reachable from the first
    true pixel using four-connected motion.
    """
    positions = np.argwhere(binary_mask)

    if len(positions) == 0:
        return 0.0

    height, width = binary_mask.shape
    start_row, start_col = positions[0]

    visited = np.zeros_like(binary_mask, dtype=bool)
    visited[start_row, start_col] = True

    queue = deque([
        (int(start_row), int(start_col))
    ])

    reachable_count = 0

    while queue:
        row, col = queue.popleft()
        reachable_count += 1

        for dr, dc in [
            (-1, 0),
            (1, 0),
            (0, -1),
            (0, 1),
        ]:
            nr = row + dr
            nc = col + dc

            if (
                0 <= nr < height
                and 0 <= nc < width
                and binary_mask[nr, nc]
                and not visited[nr, nc]
            ):
                visited[nr, nc] = True
                queue.append((nr, nc))

    total_count = int(np.count_nonzero(binary_mask))
    return reachable_count / total_count


def main():
    parser = argparse.ArgumentParser(
        description="Build a safe navigation mask from a ROS map"
    )

    default_yaml = (
        Path(__file__).resolve().parent.parent
        / "outputs"
        / "synthetic_maze_001"
        / "synthetic_maze_001.yaml"
    )

    parser.add_argument(
        "--map-yaml",
        type=Path,
        default=default_yaml,
        help="Path to the ROS map YAML file",
    )

    parser.add_argument(
        "--robot-radius",
        type=float,
        default=0.12,
        help="Approximate robot radius in metres",
    )

    parser.add_argument(
        "--safety-margin",
        type=float,
        default=0.08,
        help="Additional obstacle clearance in metres",
    )

    args = parser.parse_args()

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

    image = cv2.imread(
        str(image_path),
        cv2.IMREAD_GRAYSCALE
    )

    if image is None:
        raise RuntimeError(
            f"Could not read map image: {image_path}"
        )

    resolution = float(metadata["resolution"])

    free, occupied, unknown = classify_ros_map(image)

    # OpenCV distanceTransform measures distance from each nonzero
    # pixel to the nearest zero pixel.
    #
    # Therefore:
    # free pixel = 1
    # obstacle or unknown pixel = 0
    #
    # Unknown space is deliberately considered unsafe.
    distance_input = free.astype(np.uint8)

    clearance_px = cv2.distanceTransform(
        distance_input,
        cv2.DIST_L2,
        5,
    )

    clearance_m = clearance_px * resolution

    safe_radius = (
        args.robot_radius
        + args.safety_margin
    )

    safe_mask = free & (clearance_m >= safe_radius)

    free_count = int(np.count_nonzero(free))
    safe_count = int(np.count_nonzero(safe_mask))

    if free_count == 0:
        raise RuntimeError("Map contains no free pixels")

    safe_fraction = safe_count / free_count
    connectivity = reachable_ratio(safe_mask)

    output_dir = yaml_path.parent

    mask_path = output_dir / "navigation_mask.png"
    clearance_path = output_dir / "clearance_metres.npy"
    figure_path = output_dir / "safety_analysis.png"

    # Save mask as 0/255 for later processing.
    cv2.imwrite(
        str(mask_path),
        safe_mask.astype(np.uint8) * 255
    )

    # Save the actual floating-point clearance values.
    np.save(
        clearance_path,
        clearance_m.astype(np.float32)
    )

    # Visualization:
    # black = obstacle/unknown
    # gray  = geometrically free but unsafe for robot centre
    # green = safe robot-centre positions
    navigation_rgb = np.zeros(
        (*image.shape, 3),
        dtype=np.uint8
    )

    navigation_rgb[free] = [150, 150, 150]
    navigation_rgb[safe_mask] = [70, 200, 90]

    clearance_display = np.ma.masked_where(
        ~free,
        clearance_m
    )

    figure, axes = plt.subplots(
        1,
        3,
        figsize=(16, 5)
    )

    axes[0].imshow(
        image,
        cmap="gray",
        vmin=0,
        vmax=255
    )
    axes[0].set_title("Original occupancy map")

    clearance_plot = axes[1].imshow(
        clearance_display,
        cmap="viridis"
    )
    axes[1].set_title("Obstacle clearance (m)")
    figure.colorbar(
        clearance_plot,
        ax=axes[1],
        fraction=0.046,
        pad=0.04
    )

    axes[2].imshow(navigation_rgb)
    axes[2].set_title(
        f"Safe navigation mask\n"
        f"required clearance = {safe_radius:.2f} m"
    )

    for axis in axes:
        axis.set_xlabel("pixel column")
        axis.set_ylabel("pixel row")

    figure.tight_layout()
    figure.savefig(
        figure_path,
        dpi=160
    )
    plt.close(figure)

    height_px, width_px = image.shape

    print("===== NAVIGATION SAFETY ANALYSIS =====")
    print(f"Map: {yaml_path}")
    print(f"Pixels: {width_px} x {height_px}")
    print(f"Resolution: {resolution:.3f} m/pixel")
    print(f"Robot radius: {args.robot_radius:.3f} m")
    print(f"Safety margin: {args.safety_margin:.3f} m")
    print(f"Required clearance: {safe_radius:.3f} m")
    print()
    print(f"Original free pixels: {free_count}")
    print(f"Safe centre pixels: {safe_count}")
    print(
        f"Retained safe area: "
        f"{safe_fraction * 100.0:.2f}%"
    )
    print(
        f"Safe-space connectivity: "
        f"{connectivity * 100.0:.2f}%"
    )
    print()
    print(f"Navigation mask: {mask_path}")
    print(f"Clearance data: {clearance_path}")
    print(f"Visualization: {figure_path}")


if __name__ == "__main__":
    main()