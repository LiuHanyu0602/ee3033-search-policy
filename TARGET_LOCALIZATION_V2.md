# Target Localization V2 — Algorithm, Interfaces, and Handoff

## 1. Purpose and status

This branch upgrades the EE3033 target-coordinate estimator from a single-frame
range lookup to a robust RGB + 2-D LiDAR + multi-view localization pipeline.
The assessed target is assumed to be static and placed on the ground in a 2-D
maze. The final reported quantity is the target position in the ROS `map` frame.

Current status:

- Hardware-independent geometry is implemented.
- Seven deterministic unit tests pass on Ubuntu 22.04 / ROS 2 Humble.
- The upgraded ROS 2 adapter is implemented and compiles.
- Existing output topics remain compatible.
- Hardware-dependent calibration, topic verification, timing validation, and
  physical accuracy evaluation are **not yet complete**.
- Do not claim physical localization accuracy until the validation procedure in
  Section 11 has been completed.

## 2. Files added or changed

| Path | Purpose |
|---|---|
| `ee3033_search_policy/target_localization_core.py` | Pure NumPy geometry, LiDAR gating, triangulation, RANSAC, and robust fusion. It has no ROS dependency. |
| `ee3033_search_policy/target_localizer_node.py` | ROS 2 adapter that consumes detector/camera/LiDAR/TF data and publishes map-frame estimates. |
| `test/test_target_localization_core.py` | Seven deterministic offline unit tests. |
| `TARGET_LOCALIZATION_V2.md` | This engineering handoff and validation document. |

The local backup file
`ee3033_search_policy/target_localizer_node.py.pre_multiview` is for recovery
only and should **not** be committed.

## 3. Why the previous implementation was insufficient

The previous localizer:

1. used the bounding-box centre instead of the target/ground contact point;
2. did not explicitly remove lens distortion;
3. accepted a LiDAR return in the target bearing even when the laser hit the
   wall behind a short object;
4. used one observation only;
5. did not publish localization uncertainty;
6. assumed a fixed camera pitch even though a balancing robot rocks in pitch.

These effects create systematic range and map-coordinate errors, especially for
small floor objects and distant targets.

## 4. Implemented algorithm

### 4.1 Ground-contact pixel

For a detector box `(center_x, center_y, width, height)`, localization uses the
bottom-centre pixel rather than the box centre:

```text
u = center_x
v = center_y + height / 2
```

This approximates the physical contact point between a floor object and the
ground. A future segmentation adapter may replace this approximation with the
lowest valid mask point.

### 4.2 Camera distortion correction and bearing ray

`sensor_msgs/msg/CameraInfo` supplies `K` and `D`. For `plumb_bob` distortion,
the normalized distorted pixel is iteratively inverted using radial and
tangential coefficients:

```text
D = [k1, k2, p1, p2, k3, k4, k5, k6]
```

The resulting optical-frame ray follows the ROS optical convention:

- `+x`: image right;
- `+y`: image down;
- `+z`: camera forward.

The horizontal image bearing is:

```text
bearing = atan2(ray_x, ray_z)
```

### 4.3 Ground-plane range

For a camera at height `h` and total downward ray angle `alpha`, the horizontal
range is:

```text
r_ground = h / tan(alpha)
```

where `alpha` includes:

- calibrated static camera downward pitch;
- optional live robot pitch from IMU;
- vertical pixel ray angle.

The estimate is rejected when the ray is nearly parallel to the ground or the
result is outside configured range limits.

### 4.4 Robust LiDAR association

LiDAR samples are selected inside a circular angular window around the visual
bearing. Invalid ranges are removed. The implementation then uses the median and
median absolute deviation (MAD) to reject range outliers.

This is more stable than selecting one LaserScan beam.

### 4.5 LiDAR/ground consistency gate

A short target may sit below the 2-D LiDAR plane, causing the scan to measure
the wall behind it. When both estimates exist, LiDAR is accepted only if:

```text
abs(r_lidar - r_ground) <= lidar_ground_gate_m
```

Otherwise the LiDAR result is labeled as rejected and the ground estimate is
used. The output method records this decision.

### 4.6 Map-frame observation

For robot map pose `(x_r, y_r, yaw_r)`, camera yaw offset `yaw_c`, and image
bearing `beta`, the map-frame unit bearing is:

```text
d = [cos(yaw_r + yaw_c + beta), sin(yaw_r + yaw_c + beta)]
```

A range observation produces:

```text
p = [x_r, y_r] + range * d
```

Each observation stores the robot origin, bearing direction, range-derived map
point, and confidence weight.

### 4.7 Multi-view weighted triangulation

For origins `o_i`, unit directions `d_i`, and weights `w_i`, the target is the
weighted least-squares intersection of bearing lines:

```text
p* = argmin_p sum_i w_i ||(I - d_i d_i^T)(p - o_i)||^2
```

Near-singular geometry is rejected. Multi-view mode is only attempted after:

- enough valid observations exist; and
- the maximum camera baseline exceeds `minimum_baseline_m`.

### 4.8 RANSAC

Two bearing lines are sampled repeatedly. A candidate intersection is evaluated
using perpendicular distance to every bearing line. Observations below
`ransac_threshold_m` are inliers. The best inlier set is refitted using weighted
least squares.

Intersections behind any selected camera are rejected.

### 4.9 Robust point fusion

Range-derived points are first gated around their component-wise median. The
remaining points are combined using a weighted average. If a valid multi-view
triangulation exists, it is included with a weight proportional to the total
weight of its bearing inliers.

The detector weight currently uses:

```text
w = similarity^2 * sqrt(bounding_box_area / image_area)
```

The estimator also outputs a 2-D empirical covariance. This covariance must be
treated as a consistency indicator, not a calibrated statistical guarantee,
until physical validation is complete.

## 5. Input interfaces

| Topic | Type | Use |
|---|---|---|
| `/camera_info` | `sensor_msgs/msg/CameraInfo` | Intrinsic matrix, distortion, image dimensions. |
| `/scan` | `sensor_msgs/msg/LaserScan` | Optional bearing-associated range. |
| `/imu` | `sensor_msgs/msg/Imu` | Optional live pitch correction; disabled by default. |
| `/detector/status` | `std_msgs/msg/String` | Requires detector state `TRACKING`. |
| `/detector/target_info` | `balance_interface/msg/TargetInfo` | Detection flag, box, and similarity. |
| `/target_found` | `std_msgs/msg/Bool` | Latches the current best estimate as the final target position. |
| `/tf`, `/tf_static` | TF2 | Current `map -> base_footprint` transform. |

## 6. Output and service interfaces

| Interface | Type | Meaning |
|---|---|---|
| `/target_position_candidate` | `geometry_msgs/msg/PointStamped` | Continuously updated best estimate. |
| `/target_position` | `geometry_msgs/msg/PointStamped` | Latched final estimate after target confirmation. |
| `/target_position_with_covariance` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Final position plus 2-D covariance. |
| `/target_confidence` | `std_msgs/msg/Float32` | Detector similarity associated with latest result. |
| `/target_position_method` | `std_msgs/msg/String` | Fusion and range-source diagnostic. |
| `/target_observation_count` | `std_msgs/msg/UInt32` | Number of retained independent observations. |
| `/target_localizer/reset` | `std_srvs/srv/Trigger` | Clears observations and the previous estimate. |

Possible method strings include:

```text
robust_range_fusion:ground_plane
robust_range_fusion:lidar_ground_consistent
robust_range_fusion:ground_lidar_rejected
multiview_ransac_fused:lidar_ground_consistent
```

## 7. Parameters to add to `config/search_policy.yaml`

Add the following under `target_localizer.ros__parameters`:

```yaml
imu_topic: /imu
use_imu_pitch: false
imu_pitch_sign: 1.0
imu_pitch_offset_rad: 0.0
imu_timeout_s: 0.25
minimum_lidar_points: 2
lidar_ground_gate_m: 0.35
max_observations: 30
minimum_multiview_observations: 3
minimum_baseline_m: 0.25
ransac_threshold_m: 0.20
fusion_gate_m: 0.35
```

Keep `use_imu_pitch: false` until the physical IMU interface is verified.

## 8. Offline verification completed

Command:

```bash
python3 -m compileall ee3033_search_policy test
python3 -m pytest test/test_target_localization_core.py -v
```

Verified cases:

1. bounding-box bottom-centre contact pixel;
2. principal-point optical ray;
3. known ray/ground intersection;
4. robust LiDAR median and wall-return rejection;
5. exact weighted bearing intersection;
6. RANSAC rejection of a bad bearing;
7. robust rejection of a distant point outlier.

Latest offline result:

```text
7 passed
```

These tests validate mathematical behavior only. They do not validate physical
extrinsics, sensor timing, detector bias, AMCL error, or final field accuracy.

## 9. Known limitations

### 9.1 Detection messages are not timestamped

`balance_interface/msg/TargetInfo` has no `std_msgs/Header`. Therefore the node
cannot request `map -> base_footprint` at the exact image exposure time and uses
the latest TF. Motion can produce coordinate bias.

Recommended future interface:

```text
StampedTargetDetection
  Header header
  bool detected
  float32 similarity
  int32 center_x
  int32 center_y
  int32 width
  int32 height
```

The detector or an adapter should copy the source image timestamp and optical
frame into this message.

### 9.2 Full camera-to-base extrinsic is not yet used

Current runtime parameters model height, yaw, and downward pitch. The general
`ray_ground_intersection()` function already supports a full rotation and
translation, but the ROS adapter does not yet obtain a timestamped
`base_footprint <- camera_optical_frame` transform for every observation.

This should replace scalar approximations after the real TF tree and optical
frame are confirmed.

### 9.3 IMU correction is unverified

The IMU topic, orientation frame, pitch sign, pitch zero, timestamp quality, and
filtering must be measured on the physical balancing robot. Enabling it without
verification can reduce accuracy.

### 9.4 Multi-view baseline may be too small

The current target checker confirms after three frames and the search manager
then stops navigation. Three frames may come from nearly the same pose, so the
multi-view baseline requirement may not be reached. In that case the system
correctly falls back to robust range-point fusion.

A later mission state such as `TARGET_REFINEMENT` should deliberately collect a
second safe viewpoint before publishing the final report.

### 9.5 Segmentation is not available

The detector supplies a bounding box only. The bottom-centre pixel is an
approximation. A segmentation mask or dedicated ground-contact keypoint would
reduce pixel-to-ground bias.

## 10. Work that can still be done without the robot

- Add a stamped normalized detection message and adapter contract.
- Add synthetic timestamp/TF tests.
- Add tests for distortion, degenerate parallel rays, points behind cameras,
  covariance behavior, reset behavior, and parameter validation.
- Add a recorded-data evaluator accepting CSV/NPZ observations and producing
  mean error, median error, RMSE, 95th percentile, bias, and failure rate.
- Add a `TARGET_REFINEMENT` state-machine design in navigation-disabled mode.
- Add GitHub Actions for compile and pure-Python unit tests.

## 11. Work requiring the physical robot

Perform in this order:

1. Confirm the real topics and rates for CameraInfo, LaserScan, IMU, detection,
   odometry, AMCL, and TF.
2. Save and inspect one CameraInfo message. Confirm `distortion_model`, `K`,
   `D`, resolution, and that it matches the active camera mode.
3. Confirm `camera_link` versus `camera_optical_frame` and the complete TF chain.
4. Measure or calibrate camera height and camera-to-base roll/pitch/yaw.
5. Determine the IMU pitch sign and zero offset while the robot is upright.
6. Measure LiDAR scan-plane height and check whether each possible assessment
   target physically intersects the scan plane.
7. Place a target at surveyed ground-truth map positions and collect stationary
   observations over multiple ranges and bearings.
8. Repeat while the balancing robot is active to quantify pitch-induced error.
9. Tune `lidar_ground_gate_m`, `ransac_threshold_m`, and `fusion_gate_m` only on
   calibration trials, then evaluate on separate test positions.
10. Test target-confirmation cancellation and any refinement movement at low
    speed with an emergency stop available.

## 12. Required evaluation metrics

For ground truth `(x_gt, y_gt)` and estimate `(x, y)`:

```text
e = sqrt((x - x_gt)^2 + (y - y_gt)^2)
```

Report at least:

- mean and median Euclidean error;
- RMSE;
- 95th-percentile error;
- x/y signed bias;
- localization success rate;
- method usage counts;
- range and bearing stratification;
- stationary versus balancing/moving results.

Compare these ablations:

1. old single-frame implementation;
2. ground projection only;
3. ungated LiDAR association;
4. gated ground + LiDAR;
5. robust multi-frame fusion;
6. multi-view RANSAC fusion;
7. optional IMU pitch correction.

## 13. Safety and deployment rule

The localizer does not publish `/cmd_vel`. However, target confirmation can
change mission state and cancel navigation. Keep navigation disabled while
validating topics, TF, calibration, and coordinate estimates. Enable physical
movement only after dry-run outputs are inside the map and the Nav2 safety checks
have passed.

## Ubuntu development-machine dependency note

The ROS adapter imports the robot-specific message
`balance_interface/msg/TargetInfo`.

The `balance_interface` package is available inside the Raspberry Pi
Docker workspace, but it is not currently installed in the Ubuntu
development workspace.

Therefore, launching `target_localizer` locally may produce:

    ModuleNotFoundError: No module named 'balance_interface'

This is an environment dependency, not a failure of the
hardware-independent localization algorithms.

Offline validation should use the pure NumPy unit tests. Full ROS-node
startup must be tested inside the Raspberry Pi Docker container, or
after copying and building the genuine `balance_interface` package
locally.

Do not create a fake `balance_interface` package because its message
definition might differ from the real robot interface.
