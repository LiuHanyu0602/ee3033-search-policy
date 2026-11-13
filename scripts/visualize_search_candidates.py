#!/usr/bin/env python3
"""Offline overlay of generated viewpoints and Nav2 planning failures."""

import argparse
import json
import math
from pathlib import Path
import re

import numpy as np
import yaml

from ee3033_search_policy.map_utils import (
    GridGeometry,
    classify_cells,
    world_to_grid,
)
from ee3033_search_policy.runtime_viewpoints import (
    clearance_map,
    generate_viewpoints_from_grid,
    reachable_component,
)
from ee3033_search_policy.workflow_support import _read_pgm


FAILURE_RE = re.compile(r'Candidate\s+(\d+)\s+failed:\s*(.*)')
GENERATED_RE = re.compile(r'Generated\s+(\d+)\s+candidate poses')
SELECTED_RE = re.compile(r'Selected\s+(\d+)\s+by\s+([^;]+);\s*path=([0-9.]+)m')


def load_saved_map(yaml_path):
    yaml_path = Path(yaml_path).resolve()
    metadata = yaml.safe_load(yaml_path.read_text(encoding='utf-8'))
    image_path = Path(str(metadata['image']))
    if not image_path.is_absolute():
        image_path = yaml_path.parent/image_path
    width, height, maxval, pixels = _read_pgm(image_path)
    resolution = float(metadata['resolution'])
    origin = tuple(float(value) for value in metadata['origin'])
    negate = int(metadata.get('negate', 0))
    occupied = float(metadata['occupied_thresh'])
    free = float(metadata['free_thresh'])
    if (str(metadata.get('mode', 'trinary')).casefold() != 'trinary' or
            negate not in (0, 1) or len(origin) != 3 or
            not 0 <= free < occupied <= 1):
        raise ValueError('Expected a valid trinary ROS map')
    image = np.asarray(pixels, dtype=float).reshape(height, width)
    probability = image/maxval if negate else 1.0-image/maxval
    values_top = np.full((height, width), -1, dtype=np.int8)
    values_top[probability > occupied] = 100
    values_top[probability < free] = 0
    # Saved PGM rows run top-to-bottom; OccupancyGrid rows run from local -y/+y
    # origin upward. This reproduces the array consumed by the runtime generator.
    values = np.flipud(values_top)
    geometry = GridGeometry(width, height, resolution, *origin)
    return values, geometry, image_path


def parse_search_log(path):
    failures = {}
    generated = None
    selected = []
    for line in Path(path).read_text(encoding='utf-8', errors='replace').splitlines():
        match = FAILURE_RE.search(line)
        if match:
            failures.setdefault(int(match.group(1)), []).append(match.group(2).strip())
        match = GENERATED_RE.search(line)
        if match:
            generated = int(match.group(1))
        match = SELECTED_RE.search(line)
        if match:
            selected.append((int(match.group(1)), match.group(2), float(match.group(3))))
    return generated, failures, selected


def local_xy(world_x, world_y, geometry):
    dx, dy = world_x-geometry.origin_x, world_y-geometry.origin_y
    c, s = math.cos(geometry.origin_yaw), math.sin(geometry.origin_yaw)
    return c*dx+s*dy, -s*dx+c*dy


def main():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap

    parser = argparse.ArgumentParser(
        description='Visualize runtime candidate poses and recorded Nav2 failures')
    parser.add_argument('--map-yaml', required=True, type=Path)
    parser.add_argument('--search-log', required=True, type=Path)
    parser.add_argument('--run-json', required=True, type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--spacing-m', type=float, default=.75)
    parser.add_argument('--clearance-m', type=float, default=.18)
    parser.add_argument('--connectivity-clearance-m', type=float, default=.15)
    args = parser.parse_args()

    values, geometry, image_path = load_saved_map(args.map_yaml)
    candidates = generate_viewpoints_from_grid(
        values, geometry, spacing_m=args.spacing_m,
        clearance_m=args.clearance_m, camera_fov_deg=70.,
        camera_range_m=3., headings_per_position=4,
        free_threshold=20, occupied_threshold=65)
    generated, failures, selected = parse_search_log(args.search_log)
    invalid = sorted(index for index in failures if not 0 <= index < len(candidates))
    if invalid:
        raise ValueError(f'Log contains candidate indices outside regenerated set: {invalid}')
    if generated is not None and generated != len(candidates):
        raise ValueError(
            f'Candidate reproduction mismatch: log={generated}, regenerated={len(candidates)}')
    run = json.loads(args.run_json.read_text(encoding='utf-8'))
    home = run.get('home') or run.get('mapping_start')
    if not isinstance(home, dict):
        raise ValueError('Run JSON has no home or mapping_start pose')
    start_world = (float(home['x']), float(home['y']))
    start_cell = world_to_grid(*start_world, geometry)

    free, _, _ = classify_cells(values, 20, 65)
    clearance = clearance_map(free, geometry.resolution)
    safe = free & (clearance >= args.connectivity_clearance_m)
    reachable = reachable_component(safe, start_cell)
    position_reachable = reachable[candidates.rows, candidates.cols]

    # 0 blocked/unknown, 1 free but below candidate clearance, 2 safe but
    # disconnected from start, 3 safe and connected to start.
    background = np.zeros(values.shape, dtype=np.uint8)
    background[free] = 1
    background[safe] = 2
    background[reachable] = 3
    cmap = ListedColormap(['#171717', '#bdbdbd', '#f2cf66', '#8bd17c'])
    extent = (0, geometry.width*geometry.resolution,
              0, geometry.height*geometry.resolution)
    figure, axes = plt.subplots(1, 2, figsize=(15, 7), constrained_layout=True)
    for axis in axes:
        axis.imshow(background, origin='lower', extent=extent,
                    interpolation='nearest', cmap=cmap, vmin=0, vmax=3)
        axis.set_aspect('equal')
        axis.set_xlabel('map-local x (m)')
        axis.set_ylabel('map-local y (m)')
    local_x = (candidates.cols+.5)*geometry.resolution
    local_y = (candidates.rows+.5)*geometry.resolution
    unique = np.arange(0, len(candidates), 4)
    axes[0].scatter(local_x[unique], local_y[unique], s=18, c='#2455d6',
                    label='generated position', zorder=3)
    sx, sy = local_xy(*start_world, geometry)
    for axis in axes:
        axis.scatter([sx], [sy], marker='*', s=180, c='#ff2d2d',
                     edgecolors='white', linewidths=.8, label='search start', zorder=6)
    failed_indices = sorted(failures)
    for index in failed_indices:
        x, y = local_x[index], local_y[index]
        local_yaw = float(candidates.headings[index])-geometry.origin_yaw
        axes[1].arrow(x, y, .16*math.cos(local_yaw), .16*math.sin(local_yaw),
                      width=.012, head_width=.07, color='#d7191c', zorder=5)
        axes[1].annotate(str(index), (x, y), xytext=(3, 3),
                         textcoords='offset points', fontsize=7, color='#7f0000')
    axes[0].set_title('All generated positions and start connectivity')
    axes[1].set_title('Nav2 planning failures (red arrows show pose yaw)')
    axes[0].legend(loc='best')
    axes[1].legend(loc='best')
    output = args.output or args.map_yaml.with_name(args.map_yaml.stem+'_candidate_diagnostic.png')
    output = output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.suptitle(
        f'{args.map_yaml.name} | origin={geometry.origin_x:.2f},'
        f'{geometry.origin_y:.2f},{geometry.origin_yaw:.2f} | '
        f'candidate clearance={args.clearance_m:.2f} m | '
        f'connectivity clearance={args.connectivity_clearance_m:.2f} m')
    figure.savefig(output, dpi=180)
    plt.close(figure)

    report_path = output.with_suffix('.txt')
    lines = [
        f'map={args.map_yaml.resolve()}',
        f'image={image_path.resolve()}',
        f'start_world=({start_world[0]:.3f}, {start_world[1]:.3f})',
        f'start_cell={start_cell}',
        f'start_safe={bool(safe[start_cell])}',
        f'generated_candidates={len(candidates)}',
        f'unique_positions={len(unique)}',
        f'candidate_orientations_reachable_from_start={int(position_reachable.sum())}',
        f'failed_candidate_indices={failed_indices}',
        f'selected={selected}',
    ]
    for index in failed_indices:
        lines.append(
            f'candidate={index} world=({candidates.world_x[index]:.3f},'
            f'{candidates.world_y[index]:.3f}) yaw={candidates.headings[index]:.3f} '
            f'clearance={candidates.clearance_m[index]:.3f} '
            f'reachable={bool(position_reachable[index])} '
            f'reason={" | ".join(failures[index])}')
    report_path.write_text('\n'.join(lines)+'\n', encoding='utf-8')
    print(f'PNG: {output}')
    print(f'report: {report_path}')
    print('\n'.join(lines[:10]))


if __name__ == '__main__':
    main()
