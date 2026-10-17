#!/usr/bin/env python3

import argparse
from collections import deque
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
import yaml


FREE_VALUE = 254
OCCUPIED_VALUE = 0


def cell_bounds(row, col, corridor_px, wall_px):
    """Return pixel bounds of one free maze cell."""
    y0 = wall_px + row * (corridor_px + wall_px)
    x0 = wall_px + col * (corridor_px + wall_px)

    y1 = y0 + corridor_px
    x1 = x0 + corridor_px

    return y0, y1, x0, x1


def carve_connection(
    image,
    cell_a,
    cell_b,
    corridor_px,
    wall_px,
):
    """Remove the wall between two neighbouring maze cells."""
    row_a, col_a = cell_a
    row_b, col_b = cell_b

    y0_a, y1_a, x0_a, x1_a = cell_bounds(
        row_a, col_a, corridor_px, wall_px
    )
    y0_b, y1_b, x0_b, x1_b = cell_bounds(
        row_b, col_b, corridor_px, wall_px
    )

    # Horizontal neighbours: remove a vertical wall.
    if row_a == row_b:
        wall_x0 = min(x1_a, x1_b)
        wall_x1 = wall_x0 + wall_px

        image[
            y0_a:y1_a,
            wall_x0:wall_x1
        ] = FREE_VALUE

    # Vertical neighbours: remove a horizontal wall.
    elif col_a == col_b:
        wall_y0 = min(y1_a, y1_b)
        wall_y1 = wall_y0 + wall_px

        image[
            wall_y0:wall_y1,
            x0_a:x1_a
        ] = FREE_VALUE

    else:
        raise ValueError("Cells are not direct neighbours")


def generate_maze(
    rows,
    cols,
    corridor_px,
    wall_px,
    loop_probability,
    rng,
):
    """
    Generate a connected maze with randomized DFS.

    DFS first creates a spanning tree, guaranteeing that every cell
    can be reached. Extra walls are then opened with loop_probability
    to produce alternative routes.
    """
    height = rows * corridor_px + (rows + 1) * wall_px
    width = cols * corridor_px + (cols + 1) * wall_px

    image = np.full(
        (height, width),
        OCCUPIED_VALUE,
        dtype=np.uint8
    )

    # Create the free interior of every maze cell.
    for row in range(rows):
        for col in range(cols):
            y0, y1, x0, x1 = cell_bounds(
                row, col, corridor_px, wall_px
            )
            image[y0:y1, x0:x1] = FREE_VALUE

    visited = np.zeros((rows, cols), dtype=bool)

    start = (
        int(rng.integers(0, rows)),
        int(rng.integers(0, cols))
    )

    stack = [start]
    visited[start] = True

    tree_edges = set()

    directions = [
        (-1, 0),
        (1, 0),
        (0, -1),
        (0, 1),
    ]

    # Randomized depth-first search.
    while stack:
        current = stack[-1]
        row, col = current

        unvisited_neighbours = []

        for dr, dc in directions:
            nr = row + dr
            nc = col + dc

            if (
                0 <= nr < rows
                and 0 <= nc < cols
                and not visited[nr, nc]
            ):
                unvisited_neighbours.append((nr, nc))

        if not unvisited_neighbours:
            stack.pop()
            continue

        next_cell = unvisited_neighbours[
            int(rng.integers(0, len(unvisited_neighbours)))
        ]

        edge = tuple(sorted((current, next_cell)))
        tree_edges.add(edge)

        carve_connection(
            image,
            current,
            next_cell,
            corridor_px,
            wall_px,
        )

        visited[next_cell] = True
        stack.append(next_cell)

    # Add optional loops by removing some remaining internal walls.
    all_internal_edges = []

    for row in range(rows):
        for col in range(cols):
            if col + 1 < cols:
                all_internal_edges.append(
                    tuple(sorted(((row, col), (row, col + 1))))
                )

            if row + 1 < rows:
                all_internal_edges.append(
                    tuple(sorted(((row, col), (row + 1, col))))
                )

    extra_connections = 0

    for edge in all_internal_edges:
        if edge in tree_edges:
            continue

        if rng.random() < loop_probability:
            carve_connection(
                image,
                edge[0],
                edge[1],
                corridor_px,
                wall_px,
            )
            extra_connections += 1

    return image, len(tree_edges), extra_connections


def connectivity_ratio(image):
    """
    Calculate the proportion of free pixels reachable from one
    initial free pixel using four-connected flood fill.
    """
    free_mask = image == FREE_VALUE
    free_positions = np.argwhere(free_mask)

    if len(free_positions) == 0:
        return 0.0

    start_row, start_col = free_positions[0]

    visited = np.zeros_like(free_mask, dtype=bool)
    visited[start_row, start_col] = True

    queue = deque([(int(start_row), int(start_col))])
    reachable = 0

    height, width = image.shape

    while queue:
        row, col = queue.popleft()
        reachable += 1

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
                and free_mask[nr, nc]
                and not visited[nr, nc]
            ):
                visited[nr, nc] = True
                queue.append((nr, nc))

    total_free = int(np.count_nonzero(free_mask))
    return reachable / total_free


def main():
    parser = argparse.ArgumentParser(
        description="Generate a connected synthetic ROS maze map"
    )

    parser.add_argument("--rows", type=int, default=6)
    parser.add_argument("--cols", type=int, default=8)
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--corridor-width", type=float, default=0.75)
    parser.add_argument("--wall-width", type=float, default=0.10)
    parser.add_argument("--loop-probability", type=float, default=0.12)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--name", type=str, default="synthetic_maze_001")

    args = parser.parse_args()

    if args.rows < 2 or args.cols < 2:
        raise ValueError("rows and cols must both be at least 2")

    if args.resolution <= 0:
        raise ValueError("resolution must be positive")

    if not 0.0 <= args.loop_probability <= 1.0:
        raise ValueError("loop_probability must be between 0 and 1")

    corridor_px = max(
        1,
        int(round(args.corridor_width / args.resolution))
    )

    wall_px = max(
        1,
        int(round(args.wall_width / args.resolution))
    )

    rng = np.random.default_rng(args.seed)

    image, tree_connections, extra_connections = generate_maze(
        rows=args.rows,
        cols=args.cols,
        corridor_px=corridor_px,
        wall_px=wall_px,
        loop_probability=args.loop_probability,
        rng=rng,
    )

    ratio = connectivity_ratio(image)

    if ratio < 1.0:
        raise RuntimeError(
            f"Maze connectivity test failed: {ratio:.6f}"
        )

    script_path = Path(__file__).resolve()
    project_dir = script_path.parent.parent
    output_dir = project_dir / "outputs" / args.name
    output_dir.mkdir(parents=True, exist_ok=True)

    pgm_path = output_dir / f"{args.name}.pgm"
    yaml_path = output_dir / f"{args.name}.yaml"
    preview_path = output_dir / f"{args.name}_preview.png"

    if not cv2.imwrite(str(pgm_path), image):
        raise RuntimeError(f"Could not save {pgm_path}")

    height_px, width_px = image.shape
    width_m = width_px * args.resolution
    height_m = height_px * args.resolution

    map_metadata = {
        "image": pgm_path.name,
        "mode": "trinary",
        "resolution": float(args.resolution),
        "origin": [
            -width_m / 2.0,
            -height_m / 2.0,
            0.0,
        ],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
    }

    with yaml_path.open("w", encoding="utf-8") as file:
        yaml.safe_dump(
            map_metadata,
            file,
            sort_keys=False,
            default_flow_style=False,
        )

    plt.figure(figsize=(10, 7))
    plt.imshow(image, cmap="gray", vmin=0, vmax=255)
    plt.title(
        f"Synthetic maze: {args.rows} × {args.cols} cells\n"
        f"{width_m:.2f} m × {height_m:.2f} m, seed={args.seed}"
    )
    plt.xlabel("pixel column")
    plt.ylabel("pixel row")
    plt.tight_layout()
    plt.savefig(preview_path, dpi=160)
    plt.close()

    print("===== SYNTHETIC MAZE =====")
    print(f"Logical cells: {args.rows} x {args.cols}")
    print(f"Pixels: {width_px} x {height_px}")
    print(f"Resolution: {args.resolution:.3f} m/pixel")
    print(f"Physical size: {width_m:.2f} x {height_m:.2f} m")
    print(f"Requested corridor width: {args.corridor_width:.2f} m")
    print(f"Actual corridor width: {corridor_px * args.resolution:.2f} m")
    print(f"Requested wall width: {args.wall_width:.2f} m")
    print(f"Actual wall width: {wall_px * args.resolution:.2f} m")
    print(f"DFS tree connections: {tree_connections}")
    print(f"Extra loop connections: {extra_connections}")
    print(f"Free-space connectivity: {ratio * 100.0:.2f}%")
    print()
    print(f"PGM: {pgm_path}")
    print(f"YAML: {yaml_path}")
    print(f"Preview: {preview_path}")


if __name__ == "__main__":
    main()