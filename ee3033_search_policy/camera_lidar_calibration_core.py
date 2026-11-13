"""ROS-independent summaries for iterative camera/LiDAR alignment."""

from dataclasses import dataclass
import math
import statistics


def wrap_angle(angle):
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class CalibrationSample:
    camera_bearing_rad: float
    lidar_bearing_rad: float
    camera_range_m: float
    lidar_range_m: float
    camera_depression_rad: float
    required_depression_rad: float
    confidence: float

    def __post_init__(self):
        values = (
            self.camera_bearing_rad, self.lidar_bearing_rad,
            self.camera_range_m, self.lidar_range_m,
            self.camera_depression_rad, self.required_depression_rad,
            self.confidence,
        )
        if (not all(math.isfinite(value) for value in values) or
                min(self.camera_range_m, self.lidar_range_m) <= 0 or
                not 0 < self.confidence <= 1):
            raise ValueError('Calibration samples must be finite and positive')


@dataclass(frozen=True)
class CalibrationSummary:
    sample_count: int
    camera_bearing_rad: float
    lidar_bearing_rad: float
    camera_range_m: float
    lidar_range_m: float
    horizontal_adjustment_rad: float
    vertical_adjustment_rad: float
    range_error_m: float
    confidence: float


def _circular_median(values):
    """Median around the circular mean; calibration errors stay well below pi."""
    anchor = math.atan2(
        sum(math.sin(value) for value in values),
        sum(math.cos(value) for value in values),
    )
    return wrap_angle(anchor + statistics.median(
        wrap_angle(value-anchor) for value in values))


def summarize_samples(samples):
    """Return robust medians and physical mount adjustment directions.

    Positive horizontal adjustment means turn the physical camera left.
    Positive vertical adjustment means tilt the physical camera down.
    """
    samples = tuple(samples)
    if not samples:
        raise ValueError('At least one calibration sample is required')
    camera_bearing = _circular_median(
        [sample.camera_bearing_rad for sample in samples])
    lidar_bearing = _circular_median(
        [sample.lidar_bearing_rad for sample in samples])
    horizontal = _circular_median([
        wrap_angle(sample.camera_bearing_rad-sample.lidar_bearing_rad)
        for sample in samples
    ])
    vertical = statistics.median(
        sample.camera_depression_rad-sample.required_depression_rad
        for sample in samples)
    camera_range = statistics.median(
        sample.camera_range_m for sample in samples)
    lidar_range = statistics.median(
        sample.lidar_range_m for sample in samples)
    return CalibrationSummary(
        sample_count=len(samples),
        camera_bearing_rad=camera_bearing,
        lidar_bearing_rad=lidar_bearing,
        camera_range_m=camera_range,
        lidar_range_m=lidar_range,
        horizontal_adjustment_rad=horizontal,
        vertical_adjustment_rad=vertical,
        range_error_m=camera_range-lidar_range,
        confidence=statistics.median(sample.confidence for sample in samples),
    )
