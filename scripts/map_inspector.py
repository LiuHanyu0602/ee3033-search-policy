#!/usr/bin/env python3

from pathlib import Path
import math

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import numpy as np
import yaml


def pixel_to_world(row, col, height, resolution, origin):
    """
    Convert PGM pixel coordinates to ROS map coordinates.

    PGM:
        row increases downward
        col increases rightward

    ROS map:
        x increases rightward
        y increases upward
    """
    x0, y0, yaw = origin

    # Position of the pixel centre relative to map's bottom-left origin
    x_local = (col + 0.5) * resolution
    y_local = (height - row - 0.5) * resolution

    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)

    # Rotate and translate into ROS world coordinates
    x_world = x0 + cos_yaw * x_local - sin_yaw * y_local
    y_world = y0 + sin_yaw * x_local + cos_yaw * y_local

    return x_world, y_world


def world_to_pixel(x_world, y_world, height, resolution, origin):
    """
    Convert ROS map coordinates back to PGM pixel coordinates.
    """
    x0, y0, yaw = origin

    dx = x_world - x0
    dy = y_world - y0

    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)

    # Apply inverse rotation R^T
    x_local = cos_yaw * dx + sin_yaw * dy
    y_local = -sin_yaw * dx + cos_yaw * dy

    col = math.floor(x_local / resolution)
    row = height - 1 - math.floor(y_local / resolution)

    return row, col


def classify_trinary_map(
    image,
    occupied_thresh,
    free_thresh,
    negate
):
    """
    Classify pixels into occupied, free and unknown.

    The common saved-map convention is:
        0   -> occupied
        254 -> free
        205 -> unknown
    """
    normalized = image.astype(np.float32) / 255.0

    if negate == 0:
        occupancy_probability = 1.0 - normalized
    else:
        occupancy_probability = normalized

    # Explicitly preserve the conventional ROS unknown grey value.
    saved_unknown = (
        np.abs(image.astype(np.int16) - 205) <= 2
    )

    occupied = (
        (~saved_unknown)
        & (occupancy_probability >= occupied_thresh)
    )

    free = (
        (~saved_unknown)
        & (occupancy_probability <= free_thresh)
    )

    unknown = ~(occupied | free)

    return occupied, free, unknown


def main():
    project_dir = Path(__file__).resolve().parents[1]

    yaml_path = project_dir / "maps" / "balance_map_1.yaml"
    output_dir = project_dir / "outputs"
    output_dir.mkdir(parents=True, exist_ok=True)

    with yaml_path.open("r", encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream)

    image_path = yaml_path.parent / metadata["image"]

    image = cv2.imread(
        str(image_path),
        cv2.IMREAD_GRAYSCALE
    )

    if image is None:
        raise FileNotFoundError(
            f"Cannot load map image: {image_path}"
        )

    height, width = image.shape

    resolution = float(metadata["resolution"])
    origin = tuple(
        float(value) for value in metadata["origin"]
    )

    occupied_thresh = float(
        metadata["occupied_thresh"]
    )

    free_thresh = float(
        metadata["free_thresh"]
    )

    negate = int(metadata.get("negate", 0))

    occupied, free, unknown = classify_trinary_map(
        image,
        occupied_thresh,
        free_thresh,
        negate
    )

    print("===== ROS MAP INFORMATION =====")
    print(f"YAML: {yaml_path}")
    print(f"Image: {image_path}")
    print(f"Pixels: {width} x {height}")
    print(f"Resolution: {resolution:.3f} m/pixel")

    print(
        "Physical size: "
        f"{width * resolution:.2f} x "
        f"{height * resolution:.2f} m"
    )

    print(f"Origin: {origin}")

    print(
        f"Occupied: {occupied.sum()} "
        f"({occupied.mean() * 100:.2f}%)"
    )

    print(
        f"Free: {free.sum()} "
        f"({free.mean() * 100:.2f}%)"
    )

    print(
        f"Unknown: {unknown.sum()} "
        f"({unknown.mean() * 100:.2f}%)"
    )

    # Test coordinate conversion using the centre pixel
    test_row = height // 2
    test_col = width // 2

    test_x, test_y = pixel_to_world(
        test_row,
        test_col,
        height,
        resolution,
        origin
    )

    recovered_row, recovered_col = world_to_pixel(
        test_x,
        test_y,
        height,
        resolution,
        origin
    )

    print("\n===== COORDINATE TEST =====")
    print(
        f"Pixel ({test_row}, {test_col})"
        f" -> world ({test_x:.3f}, {test_y:.3f})"
        f" -> pixel ({recovered_row}, {recovered_col})"
    )

    assert (recovered_row, recovered_col) == (
        test_row,
        test_col
    )

    print("Coordinate round-trip test: PASSED")

    # 0 = free, 1 = unknown, 2 = occupied
    class_map = np.full(
        image.shape,
        1,
        dtype=np.uint8
    )

    class_map[free] = 0
    class_map[occupied] = 2

    map_colours = ListedColormap([
        "white",
        "#b0b0b0",
        "black"
    ])

    output_path = output_dir / "map_inspection.png"

    figure, axes = plt.subplots(
        1,
        2,
        figsize=(10, 6),
        constrained_layout=True
    )

    axes[0].imshow(
        image,
        cmap="gray",
        vmin=0,
        vmax=255
    )
    axes[0].set_title("Original PGM")

    axes[1].imshow(
        class_map,
        cmap=map_colours,
        vmin=0,
        vmax=2
    )
    axes[1].set_title(
        "Free / Unknown / Occupied"
    )

    for axis in axes:
        axis.set_xlabel("column c")
        axis.set_ylabel("row r")
        axis.grid(False)

    figure.savefig(
        output_path,
        dpi=180
    )

    plt.close(figure)

    print(f"\nSaved visualization: {output_path}")


if __name__ == "__main__":
    main()
