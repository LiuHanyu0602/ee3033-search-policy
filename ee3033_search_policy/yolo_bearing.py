"""Horizontal optical-frame bearing for raw, calibrated detector pixels."""

from dataclasses import dataclass
import math

import numpy as np

from .target_localization_core import CameraModel, undistort_normalized


@dataclass(frozen=True)
class RawCameraCalibration:
    width: int
    height: int
    camera: CameraModel

    @classmethod
    def load(cls, path):
        with np.load(path, allow_pickle=False) as data:
            k = np.asarray(data['camera_matrix'], dtype=float)
            d = np.asarray(data['dist_coeffs'], dtype=float).reshape(-1)
            width = float(data['image_width'].item())
            height = float(data['image_height'].item())
        if (k.shape != (3, 3) or d.shape != (5,) or
                not np.all(np.isfinite(k)) or not np.all(np.isfinite(d)) or
                not math.isfinite(width) or not math.isfinite(height) or
                width <= 0 or height <= 0 or
                not width.is_integer() or not height.is_integer()):
            raise ValueError('Expected finite raw calibration with K(3x3), D(5), positive image size')
        if (k[0, 0] <= 0 or k[1, 1] <= 0 or
                not np.allclose(k[2], [0, 0, 1], rtol=0, atol=1e-12) or
                abs(k[0, 1]) > 1e-12 or abs(k[1, 0]) > 1e-12 or
                not 0 <= k[0, 2] < width or not 0 <= k[1, 2] < height):
            raise ValueError('Invalid pinhole camera matrix')
        return cls(int(width), int(height), CameraModel(
            k[0, 0], k[1, 1], k[0, 2], k[1, 2], tuple(d)))

    def validate_camera_info(self, info):
        c = self.camera
        expected_k = [c.fx, 0, c.cx, 0, c.fy, c.cy, 0, 0, 1]
        if (info.width != self.width or info.height != self.height or
                not info.header.frame_id or info.distortion_model != 'plumb_bob' or
                len(info.k) != 9 or len(info.d) != 5 or
                not np.allclose(info.k, expected_k, rtol=0, atol=1e-6) or
                not np.allclose(info.d, c.distortion, rtol=0, atol=1e-8)):
            raise ValueError('CameraInfo must match the configured raw calibration (size, K, D, model)')
        roi = info.roi
        if (info.binning_x not in (0, 1) or info.binning_y not in (0, 1) or
                roi.x_offset != 0 or roi.y_offset != 0 or roi.do_rectify or
                roi.width not in (0, self.width) or roi.height not in (0, self.height)):
            raise ValueError('Cropped, binned or rectified detector pixels are not supported')

    def bearing(self, u, v):
        """Return radians, positive image-right, zero along optical +z."""
        if (not math.isfinite(u) or not math.isfinite(v) or
                not 0 <= u < self.width or not 0 <= v < self.height):
            raise ValueError('Bbox centre must be finite and inside the raw image')
        c = self.camera
        xd, yd = (u-c.cx)/c.fx, (v-c.cy)/c.fy
        x, y = undistort_normalized(xd, yd, c.distortion, iterations=100)
        k1, k2, p1, p2, k3 = c.distortion
        r2 = x*x + y*y
        radial = 1 + k1*r2 + k2*r2*r2 + k3*r2*r2*r2
        projected = (x*radial + 2*p1*x*y + p2*(r2+2*x*x),
                     y*radial + p1*(r2+2*y*y) + 2*p2*x*y)
        if (not np.all(np.isfinite(projected)) or
                max(abs(projected[0]-xd), abs(projected[1]-yd)) > 1e-7):
            raise ValueError('Pixel undistortion did not converge')
        return math.atan2(float(x), 1.0)
