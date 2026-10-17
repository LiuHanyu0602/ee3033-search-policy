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


def pixel_to_world(row, col, metadata, image_height):
    """
    Convert a map pixel centre into ROS world coordinates.
    """
    resolution = float(metadata["resolution"])
    origin_x, origin_y, origin_yaw = metadata["origin"]

    local_x = (col + 0.5) * resolution
    local_y = (image_height - row - 0.5) * resolution

    cos_yaw = math.cos(origin_yaw)
    sin_yaw = math.sin(origin_yaw)

    world_x = (
        origin_x
        + cos_yaw * local_x
        - sin_yaw * local_y
    )

    world_y = (
        origin_y
        + sin_yaw * local_x
        + cos_yaw * local_y
    )

    return world_x, world_y


def generate_candidate_positions(
    safe_mask,
    clearance_m,
    spacing_px,
):
    """
    Divide the map into blocks and select the safest pixel from
    every block containing safe space.

    This is more reliable than testing only fixed grid intersections,
    because a narrow corridor might fall between grid lines.
    """
    height, width = safe_mask.shape
    positions = []

    for row_start in range(0, height, spacing_px):
        row_end = min(row_start + spacing_px, height)

        for col_start in range(0, width, spacing_px):
            col_end = min(col_start + spacing_px, width)

            block_safe = safe_mask[
                row_start:row_end,
                col_start:col_end
            ]

            safe_pixels = np.argwhere(block_safe)

            if len(safe_pixels) == 0:
                continue

            block_clearance = clearance_m[
                row_start:row_end,
                col_start:col_end
            ]

            local_clearances = block_clearance[
                block_safe
            ]

            best_index = int(np.argmax(local_clearances))
            local_row, local_col = safe_pixels[best_index]

            global_row = row_start + int(local_row)
            global_col = col_start + int(local_col)

            positions.append((global_row, global_col))

    return positions


def wrap_angle(angle):
    """
    Wrap an angle into [-pi, pi].
    """
    return (
        angle + math.pi
    ) % (2.0 * math.pi) - math.pi


def has_line_of_sight(
    free_mask,
    start_row,
    start_col,
    end_row,
    end_col,
):
    """
    Bresenham line test.

    Return True only if every map pixel from the camera to the
    target pixel is free. An occupied or unknown pixel blocks view.
    """
    height, width = free_mask.shape

    x0 = int(start_col)
    y0 = int(start_row)
    x1 = int(end_col)
    y1 = int(end_row)

    dx = abs(x1 - x0)
    dy = abs(y1 - y0)

    step_x = 1 if x0 < x1 else -1
    step_y = 1 if y0 < y1 else -1

    error = dx - dy

    while True:
        if not (
            0 <= y0 < height
            and 0 <= x0 < width
        ):
            return False

        if not free_mask[y0, x0]:
            return False

        if x0 == x1 and y0 == y1:
            return True

        double_error = 2 * error

        if double_error > -dy:
            error -= dy
            x0 += step_x

        if double_error < dx:
            error += dx
            y0 += step_y


def raycast_visibility(
    free_mask,
    start_row,
    start_col,
    heading,
    field_of_view,
    max_range_px,
):
    """
    Calculate exact pixel visibility using:

    1. camera range;
    2. camera field of view;
    3. obstacle line-of-sight.
    """
    visible = np.zeros_like(
        free_mask,
        dtype=bool
    )

    free_rows, free_cols = np.nonzero(free_mask)

    # Pixel displacement in world-coordinate convention:
    # +x points right, +y points upward.
    delta_x = free_cols - start_col
    delta_y = start_row - free_rows

    distance_squared = (
        delta_x.astype(np.float32) ** 2
        + delta_y.astype(np.float32) ** 2
    )

    bearings = np.arctan2(
        delta_y,
        delta_x
    )

    angle_errors = np.arctan2(
        np.sin(bearings - heading),
        np.cos(bearings - heading),
    )

    geometric_condition = (
        (distance_squared <= max_range_px ** 2)
        & (
            np.abs(angle_errors)
            <= field_of_view / 2.0
        )
    )

    possible_rows = free_rows[
        geometric_condition
    ]

    possible_cols = free_cols[
        geometric_condition
    ]

    for row, col in zip(
        possible_rows,
        possible_cols
    ):
        if has_line_of_sight(
            free_mask=free_mask,
            start_row=start_row,
            start_col=start_col,
            end_row=int(row),
            end_col=int(col),
        ):
            visible[row, col] = True

    visible[start_row, start_col] = True

    return visible

def main():
    parser = argparse.ArgumentParser(
        description=(
            "Generate safe candidate viewpoints and "
            "simulate camera visibility"
        )
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
        "--spacing",
        type=float,
        default=0.75,
        help="Candidate position spacing in metres",
    )

    parser.add_argument(
        "--camera-fov-deg",
        type=float,
        default=70.0,
        help="Horizontal camera field of view in degrees",
    )

    parser.add_argument(
        "--camera-range",
        type=float,
        default=3.0,
        help="Maximum useful detection distance in metres",
    )

    args = parser.parse_args()

    yaml_path = args.map_yaml.resolve()

    if not yaml_path.exists():
        raise FileNotFoundError(
            f"Map YAML does not exist: {yaml_path}"
        )

    with yaml_path.open("r", encoding="utf-8") as file:
        metadata = yaml.safe_load(file)

    image_path = Path(metadata["image"])

    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path

    navigation_mask_path = (
        yaml_path.parent / "navigation_mask.png"
    )

    clearance_path = (
        yaml_path.parent / "clearance_metres.npy"
    )

    image = cv2.imread(
        str(image_path),
        cv2.IMREAD_GRAYSCALE
    )

    navigation_image = cv2.imread(
        str(navigation_mask_path),
        cv2.IMREAD_GRAYSCALE
    )

    if image is None:
        raise RuntimeError(
            f"Cannot read occupancy map: {image_path}"
        )

    if navigation_image is None:
        raise RuntimeError(
            "Cannot read navigation_mask.png. "
            "Run build_navigation_mask.py first."
        )

    if not clearance_path.exists():
        raise FileNotFoundError(
            "Cannot find clearance_metres.npy. "
            "Run build_navigation_mask.py first."
        )

    clearance_m = np.load(clearance_path)

    free_mask, occupied_mask, unknown_mask = (
        classify_ros_map(image)
    )

    safe_mask = navigation_image >= 250

    resolution = float(metadata["resolution"])

    spacing_px = max(
        1,
        int(round(args.spacing / resolution))
    )

    max_range_px = (
        args.camera_range / resolution
    )

    field_of_view = math.radians(
        args.camera_fov_deg
    )

    positions = generate_candidate_positions(
        safe_mask=safe_mask,
        clearance_m=clearance_m,
        spacing_px=spacing_px,
    )

    if not positions:
        raise RuntimeError(
            "No candidate positions were generated"
        )

    headings = np.array([
        0.0,
        math.pi / 2.0,
        math.pi,
        -math.pi / 2.0,
    ])

    candidate_rows = []
    candidate_cols = []
    candidate_x = []
    candidate_y = []
    candidate_headings = []
    visible_counts = []
    visible_areas = []
    visibility_masks = []

    image_height = image.shape[0]

    for row, col in positions:
        world_x, world_y = pixel_to_world(
            row,
            col,
            metadata,
            image_height,
        )

        for heading in headings:
            visible = raycast_visibility(
                free_mask=free_mask,
                start_row=row,
                start_col=col,
                heading=float(heading),
                field_of_view=field_of_view,
                max_range_px=max_range_px,
            )

            pixel_count = int(
                np.count_nonzero(visible)
            )

            area_m2 = (
                pixel_count
                * resolution
                * resolution
            )

            candidate_rows.append(row)
            candidate_cols.append(col)
            candidate_x.append(world_x)
            candidate_y.append(world_y)
            candidate_headings.append(heading)
            visible_counts.append(pixel_count)
            visible_areas.append(area_m2)
            visibility_masks.append(
                visible.astype(np.uint8)
            )

    candidate_rows = np.asarray(
        candidate_rows,
        dtype=np.int32
    )

    candidate_cols = np.asarray(
        candidate_cols,
        dtype=np.int32
    )

    candidate_x = np.asarray(
        candidate_x,
        dtype=np.float32
    )

    candidate_y = np.asarray(
        candidate_y,
        dtype=np.float32
    )

    candidate_headings = np.asarray(
        candidate_headings,
        dtype=np.float32
    )

    visible_counts = np.asarray(
        visible_counts,
        dtype=np.int32
    )

    visible_areas = np.asarray(
        visible_areas,
        dtype=np.float32
    )

    visibility_masks = np.stack(
        visibility_masks,
        axis=0
    )

    # Number of candidate poses that can see each free pixel.
    coverage_count = np.sum(
        visibility_masks,
        axis=0
    )

    observable_mask = coverage_count > 0

    free_count = int(np.count_nonzero(free_mask))
    observable_free_count = int(
        np.count_nonzero(
            observable_mask & free_mask
        )
    )

    observable_ratio = (
        observable_free_count / free_count
    )

    best_index = int(np.argmax(visible_counts))

    output_npz = (
        yaml_path.parent / "candidate_viewpoints.npz"
    )

    output_figure = (
        yaml_path.parent / "candidate_visibility.png"
    )

    np.savez_compressed(
        output_npz,
        rows=candidate_rows,
        cols=candidate_cols,
        world_x=candidate_x,
        world_y=candidate_y,
        headings=candidate_headings,
        visible_counts=visible_counts,
        visible_areas_m2=visible_areas,
        visibility_masks=visibility_masks,
        resolution=np.float32(resolution),
        camera_fov_deg=np.float32(
            args.camera_fov_deg
        ),
        camera_range_m=np.float32(
            args.camera_range
        ),
    )

    figure, axes = plt.subplots(
        1,
        3,
        figsize=(17, 5)
    )

    # Panel 1: candidate positions.
    candidate_background = np.zeros(
        (*image.shape, 3),
        dtype=np.uint8
    )

    candidate_background[free_mask] = [
        210, 210, 210
    ]

    candidate_background[safe_mask] = [
        80, 180, 100
    ]

    axes[0].imshow(candidate_background)

    position_array = np.asarray(positions)

    axes[0].scatter(
        position_array[:, 1],
        position_array[:, 0],
        s=15,
        c="red",
        label="candidate position",
    )

    axes[0].set_title(
        f"{len(positions)} candidate positions"
    )
    axes[0].legend(loc="lower right")

    # Panel 2: the single pose with the largest visible area.
    best_visible = (
        visibility_masks[best_index].astype(bool)
    )

    visibility_rgb = np.zeros(
        (*image.shape, 3),
        dtype=np.uint8
    )

    visibility_rgb[free_mask] = [
        235, 235, 235
    ]

    visibility_rgb[best_visible] = [
        80, 170, 255
    ]

    best_row = int(candidate_rows[best_index])
    best_col = int(candidate_cols[best_index])
    best_heading = float(
        candidate_headings[best_index]
    )

    axes[1].imshow(visibility_rgb)

    arrow_length = 10

    axes[1].arrow(
        best_col,
        best_row,
        arrow_length * math.cos(best_heading),
        -arrow_length * math.sin(best_heading),
        color="red",
        width=0.8,
        head_width=3.0,
        length_includes_head=True,
    )

    axes[1].set_title(
        "Example camera visibility\n"
        f"visible area = "
        f"{visible_areas[best_index]:.2f} m²"
    )

    # Panel 3: how many candidate poses can observe each pixel.
    coverage_display = np.ma.masked_where(
        ~free_mask,
        coverage_count
    )

    coverage_plot = axes[2].imshow(
        coverage_display,
        cmap="magma"
    )

    axes[2].set_title(
        "Number of poses observing each pixel"
    )

    figure.colorbar(
        coverage_plot,
        ax=axes[2],
        fraction=0.046,
        pad=0.04,
    )

    for axis in axes:
        axis.set_xlabel("pixel column")
        axis.set_ylabel("pixel row")

    figure.tight_layout()
    figure.savefig(
        output_figure,
        dpi=160
    )
    plt.close(figure)

    print("===== CANDIDATE VIEWPOINTS =====")
    print(f"Map: {yaml_path}")
    print(
        f"Candidate spacing: "
        f"{args.spacing:.2f} m "
        f"({spacing_px} pixels)"
    )
    print(
        f"Candidate positions: {len(positions)}"
    )
    print(
        f"Headings per position: {len(headings)}"
    )
    print(
        f"Total candidate poses: "
        f"{len(candidate_rows)}"
    )
    print(
        f"Camera FOV: "
        f"{args.camera_fov_deg:.1f} degrees"
    )
    print(
        f"Camera range: "
        f"{args.camera_range:.2f} m"
    )
    print()
    print(
        f"Mean visible area per pose: "
        f"{np.mean(visible_areas):.3f} m²"
    )
    print(
        f"Maximum visible area: "
        f"{np.max(visible_areas):.3f} m²"
    )
    print(
        f"Observable free space: "
        f"{observable_ratio * 100.0:.2f}%"
    )
    print()
    print(f"Saved data: {output_npz}")
    print(f"Saved figure: {output_figure}")


if __name__ == "__main__":
    main()