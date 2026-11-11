"""ROS-independent helpers for the interactive real-robot workflow."""

from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
import shutil


TERMINAL_MISSION_STATES = frozenset({
    'FINISHED', 'SEARCH_EXHAUSTED', 'LOCALIZATION_FAILED',
    'OPERATOR_STOPPED', 'MISSION_TIMEOUT', 'FAULT', 'STOP_UNVERIFIED',
})


def map_output_paths(prefix):
    prefix = Path(prefix)
    return Path(str(prefix)+'.yaml'), Path(str(prefix)+'.pgm')


@dataclass(frozen=True)
class MapCleanupResult:
    removed: int
    raw_path: Path
    raw_sha256: str
    cleaned_sha256: str


@dataclass(frozen=True)
class PlanarPose:
    x: float
    y: float
    yaw: float

    def __post_init__(self):
        if not all(math.isfinite(value) for value in (self.x, self.y, self.yaw)):
            raise ValueError('Home pose must contain finite planar values')


def validate_new_map_prefix(value):
    """Return an absolute, nonexisting map prefix without a file suffix."""
    prefix = Path(str(value)).expanduser()
    if not prefix.is_absolute():
        raise ValueError('Map prefix must be an absolute path')
    if prefix.suffix in ('.yaml', '.yml', '.pgm'):
        raise ValueError('Map prefix must not include .yaml or .pgm')
    if prefix.name in ('', '.', '..') or str(prefix) in ('/', '/home', '/tmp'):
        raise ValueError('Map prefix is too broad')
    if not prefix.parent.is_dir():
        raise ValueError(f'Map directory does not exist: {prefix.parent}')
    outputs = (*map_output_paths(prefix), Path(str(prefix)+'.raw.pgm'),
               Path(str(prefix)+'.run.json'))
    existing = [str(path) for path in outputs if path.exists()]
    if existing:
        raise ValueError('Refusing to overwrite existing map file(s): '+', '.join(existing))
    return prefix


def file_sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def saved_map_files(prefix):
    yaml_path, pgm_path = map_output_paths(prefix)
    if not yaml_path.is_file() or yaml_path.stat().st_size <= 0:
        raise ValueError(f'Map metadata missing or empty: {yaml_path}')
    if not pgm_path.is_file() or pgm_path.stat().st_size <= 0:
        raise ValueError(f'Map image missing or empty: {pgm_path}')
    expected = f'image: {pgm_path.name}'
    if expected not in yaml_path.read_text(encoding='utf-8'):
        raise ValueError(f'Map YAML does not reference {pgm_path.name}')
    return yaml_path, pgm_path


def angular_error(a, b):
    return abs(math.atan2(math.sin(a-b), math.cos(a-b)))


def _pgm_header(data):
    """Return PGM magic, width, height, maxval, and raster offset."""
    tokens = []
    index = 0
    while len(tokens) < 4:
        while index < len(data) and data[index] in b' \t\r\n':
            index += 1
        if index < len(data) and data[index] == ord('#'):
            while index < len(data) and data[index] not in b'\r\n':
                index += 1
            continue
        start = index
        while index < len(data) and data[index] not in b' \t\r\n#':
            index += 1
        if start == index:
            raise ValueError('Invalid PGM header')
        tokens.append(data[start:index].decode('ascii'))
    if index >= len(data) or data[index] not in b' \t\r\n':
        raise ValueError('PGM header is not followed by raster data')
    if data[index:index+2] == b'\r\n':
        index += 2
    else:
        index += 1
    magic, width, height, maxval = tokens
    width, height, maxval = int(width), int(height), int(maxval)
    if magic not in ('P5', 'P2') or width <= 0 or height <= 0:
        raise ValueError('Unsupported or invalid PGM geometry')
    if not 0 < maxval <= 255:
        raise ValueError('Only 8-bit PGM maps are supported')
    return magic, width, height, maxval, index


def _read_pgm(path):
    data = Path(path).read_bytes()
    magic, width, height, maxval, offset = _pgm_header(data)
    count = width*height
    if magic == 'P5':
        pixels = list(data[offset:offset+count])
    else:
        text = data[offset:].decode('ascii')
        values = []
        for line in text.splitlines():
            values.extend(line.split('#', 1)[0].split())
        pixels = [int(value) for value in values[:count]]
    if len(pixels) != count or any(not 0 <= value <= maxval for value in pixels):
        raise ValueError('PGM raster size or value is invalid')
    return width, height, maxval, pixels


def _map_thresholds(yaml_path):
    values = {}
    for raw in Path(yaml_path).read_text(encoding='utf-8').splitlines():
        line = raw.split('#', 1)[0].strip()
        if ':' not in line:
            continue
        key, value = line.split(':', 1)
        if key.strip() in ('negate', 'occupied_thresh', 'free_thresh'):
            values[key.strip()] = value.strip()
    try:
        negate = int(values['negate'])
        occupied = float(values['occupied_thresh'])
        free = float(values['free_thresh'])
    except (KeyError, ValueError) as exc:
        raise ValueError('Map YAML is missing valid occupancy thresholds') from exc
    if negate not in (0, 1) or not 0 <= free < occupied <= 1:
        raise ValueError('Map YAML occupancy thresholds are inconsistent')
    return negate, occupied, free


def clean_isolated_occupied_pixels(yaml_path, pgm_path, radius=2):
    """Remove only occupied cells whose complete 5x5 neighborhood is free.

    The original raster is retained as ``<prefix>.raw.pgm``. Unknown cells do
    not satisfy the free-neighbor test, so map boundaries are unchanged.
    """
    if radius != 2:
        raise ValueError('The validated isolated-cell filter requires radius=2')
    yaml_path, pgm_path = Path(yaml_path), Path(pgm_path)
    width, height, maxval, pixels = _read_pgm(pgm_path)
    negate, occupied_threshold, free_threshold = _map_thresholds(yaml_path)

    def probability(value):
        normalized = value/maxval
        return normalized if negate else 1.0-normalized

    occupied = [probability(value) > occupied_threshold for value in pixels]
    free = [probability(value) < free_threshold for value in pixels]
    replacements = []
    for row in range(radius, height-radius):
        for column in range(radius, width-radius):
            center = row*width+column
            if not occupied[center]:
                continue
            neighbors = []
            all_free = True
            for dy in range(-radius, radius+1):
                for dx in range(-radius, radius+1):
                    if dx == 0 and dy == 0:
                        continue
                    index = (row+dy)*width+(column+dx)
                    if not free[index]:
                        all_free = False
                        break
                    neighbors.append(pixels[index])
                if not all_free:
                    break
            if all_free and len(neighbors) == 24:
                replacements.append((center, sorted(neighbors)[len(neighbors)//2]))

    raw_path = Path(str(pgm_path.with_suffix(''))+'.raw.pgm')
    if raw_path.exists():
        raise ValueError(f'Refusing to overwrite raw map backup: {raw_path}')
    shutil.copy2(pgm_path, raw_path)
    for index, value in replacements:
        pixels[index] = value
    temporary = Path(str(pgm_path)+'.tmp')
    header = (
        f'P5\n# EE3033 isolated occupied-cell filter\n'
        f'{width} {height}\n{maxval}\n').encode('ascii')
    temporary.write_bytes(header+bytes(pixels))
    temporary.replace(pgm_path)
    return MapCleanupResult(
        removed=len(replacements),
        raw_path=raw_path,
        raw_sha256=file_sha256(raw_path),
        cleaned_sha256=file_sha256(pgm_path),
    )
