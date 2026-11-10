# Stamped target localization checkpoint

Baseline: 7616ddc, user reported 151 existing tests passed. Adds only three source/
test files and this note; does NOT replace old localizer or change navigation.

## Interfaces

Node: `python3 -m ee3033_search_policy.stamped_target_localizer_node`.
Inputs: /target/observation (TargetObservation), /target/configuration (adapter's
latched JSON), configurable camera_info_topic, TF.
Outputs: /target/estimate_candidate
(TargetEstimate, BEST_EFFORT/volatile/depth 1),
/target/localization_status (latched String).
No /target_position, /target_found, navigation action or cmd_vel output.
No setup.py entry point or full_runtime wiring in this checkpoint.

Required calibration_file parameter: verified raw calibration.npz used by adapter.
Defaults geometry_verified=false, assume_ground_contact=false. Enabling either
requires measurement evidence, not merely to make a test pass.

## Algorithms

Reuse teammate RawCameraCalibration and its inverse-distortion convergence check.
Keep bearing_rad unchanged; center angle is not a ground contact ray.
Query map <- actual camera optical frame at source-image stamp, never latest TF.
Rotate full optical rays with normalized transform quaternion; origin includes
actual camera translation. Intersect bbox bottom-center ray with configured map
plane z=ground_z_m. Reject horizon/upward/out-of-range/clipped boxes. Only enable
ground projection for objects actually touching that plane and visible contact.

Collect center-bearing rays even when ground range is unavailable. RANSAC requires
three distinct origins, >=.25 m baseline, >=10 degree ray crossing, >=3 inliers,
forward intersections and range bounds. Provisional thresholds require tuning.
Ground and bearing estimates must agree within .35 m or localization is invalid.
Robust median-gated range fusion reused from original core. Added .1 m standard
deviation floor is provisional, NOT calibrated pose/measurement covariance.
quality currently copies detection confidence: not a spatial-accuracy probability.

## Epoch/time protections

Accept only configured episode/class and increasing fresh positive image stamps.
Configuration/CameraInfo mismatch or backward ROS clock invalidates history.
Changing nonempty tracking IDs clears fusion history. Without stable detector
tracking IDs, same-class multi-object identity is NOT guaranteed: do not use
multiview with multiple ambiguous objects until tracking association is verified.
Limit history to 30 observations /15 seconds. Calibration is assumed static;
CameraInfo matches raw K/D/size/model. Cropped/rectified pixels unsupported.

## Important limitations

valid=true means a confirmed detection has a geometrically usable CANDIDATE.
It is NOT proof the robot stopped and MUST NOT directly finish a mission or start
return home. Mission coordinator must verify stop, episode and coordinate age.
No motion is requested to gather baseline. Ground-only estimate may work while
stationary; multiview needs naturally available distinct observations.

Pitch compensation requires actual time-varying camera optical TF. Fixed yaw-only
base TF does not account for balance pitch. IMU/extrinsic integration is hardware
work; this node does not silently add an unverified pitch sign or static height.
No LiDAR association in this new node yet. Existing legacy LiDAR helpers remain
untouched; do not claim LiDAR-camera fusion is wired into the typed path.
Future work: timestamped sensor fusion, TF waiting/retry without blocking, camera
model version handshake, identity consistency, mission-stop/report handoff,
launch/dependency installation and synthetic ROS replay tests.

## Local validation commands (Ubuntu, no robot)

```bash
cd ~/ee3033_ws/src/ee3033_search_policy
/usr/bin/python3 -m pytest test/test_stamped_localization.py -v
/usr/bin/python3 -m pytest test -q
```

ROS smoke check in unused domain, using existing raw calibration:

```bash
ROS_DOMAIN_ID=115 /usr/bin/python3 -m ee3033_search_policy.stamped_target_localizer_node \
  --ros-args -p calibration_file:="$PWD/results/updated_calibration/calibration.npz"
```

Expected initial status WAITING_FOR_VERIFIED_GEOMETRY. Missing data must not
produce valid estimates. Do not start old full_runtime or enable movement.
Author has no ROS runtime here: ROS startup, service wiring, sensor replay and
physical accuracy are pending local/robot acceptance.
