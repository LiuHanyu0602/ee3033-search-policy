"""ROS-independent helpers for the interactive real-robot workflow."""

from dataclasses import dataclass
import hashlib
import math
from pathlib import Path


TERMINAL_MISSION_STATES = frozenset({
    'FINISHED', 'SEARCH_EXHAUSTED', 'LOCALIZATION_FAILED',
    'OPERATOR_STOPPED', 'MISSION_TIMEOUT', 'FAULT', 'STOP_UNVERIFIED',
})


def map_output_paths(prefix):
    prefix = Path(prefix)
    return Path(str(prefix)+'.yaml'), Path(str(prefix)+'.pgm')


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
    outputs = (*map_output_paths(prefix), Path(str(prefix)+'.run.json'))
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
