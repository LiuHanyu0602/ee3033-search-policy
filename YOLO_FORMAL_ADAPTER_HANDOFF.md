# Formal YOLO adapter: phase checkpoint

Adds yolo_detection_adapter_node.py, yolo_adapter_session.py and tests.
Run using python -m from repository root, or `ros2 run ee3033_search_policy
yolo_detection_adapter` after building the package. Launch integration is pending.
Depends on rclpy, sensor_msgs, std_msgs, std_srvs and the already-built
ee3033_search_policy_interfaces package. Typed input additionally requires
yolo_msgs; NCNN JSON mode does not import it. No Ultralytics needed by the adapter.

Services: /target/set_class (SetTargetClass), /target/reset (Trigger).
Outputs: /target/observation (TargetObservation, volatile),
/target/configuration (latched String JSON), /target/available_classes (latched
String JSON), /target/adapter_status (latched String).
Inputs: parameter detections_topic (default /yolo/detections, must verify live),
camera_info_topic (default /camera_info), mission_topic (default /mission/state).

## Existing Pi NCNN JSON input

The operator verified `/yolo/detections` from `yolo_ncnn` is
`std_msgs/msg/String`, RELIABLE, VOLATILE. Its schema is:

```json
{"timestamp":{"sec":1793770649,"nanosec":722597000},"detections":[
  {"class_id":39,"class_name":"bottle","confidence":0.88330078125,
   "bbox":[240.875,16.5,365.125,311.0]}]}
```

Use `input_format:=ncnn_json` for this producer. The default remains
`input_format:=detection_array` for existing typed producers. Do not start both
message types on the same topic.

JSON has no frame ID or image size. In JSON mode the adapter also subscribes to
`image_topic` (default `/camera/image_raw`, BEST_EFFORT depth 1) and retains only
the latest 64 image metadata records, not pixel buffers. It matches timestamps
exactly in integer nanoseconds; the output header preserves the source stamp and
frame without substitution from a newer image or CameraInfo. Missing, stale or
future source metadata, unsupported encodings, malformed JSON and invalid bbox
values log explicit rejection and produce no observation. If a detection arrives
before its source metadata, it is rejected rather than guessed or queued; later
fresh matching frames recover normally.

The JSON subscription is RELIABLE depth 1. Empty detection arrays are valid
heartbeats and break partial confirmation sequences. No tracking ID is invented;
the existing IoU continuity is not proof of persistent physical identity.
Source-image geometry must agree with the validated CameraInfo; a change clears
the episode and metadata cache. The operator observed 640x480 rgb8 with matching
new K/D and frame name `default_cam`; that name alone does NOT prove a valid
optical-frame TF for map localization.

For stationary tests, set `mission_topic:=/bearing_test/mission_state` and publish
test state strings there only; do not call production `/mission/start_search`.
No search manager or localizer is required. Export the actual model label table
to `labels_file` and build/source the genuine interface package first.
The existing Pi detector script and NCNN model are unchanged.

Targeted offline tests:

```bash
python3 -m pytest test/test_yolo_json_input.py test/test_yolo_bearing.py \
  test/test_yolo_detection_core.py test/test_yolo_adapter_session.py \
  test/test_target_localization_core.py -q
```

2026-11-04 FAGIO offline checkpoint, based on `7616ddc` plus JSON changes:
102 tests passed; Python compilation and `git diff --check` passed. Tests include
the two operator-supplied NCNN samples and ROS test-double node initialization
with `yolo_msgs` unavailable. This is not a ROS build or live adapter acceptance.
The samples' bottle centres yield approximately +0.02429660 rad (+1.3921 deg)
and +0.14201882 rad (+8.1371 deg) with the saved calibration. These are computed
reference values, not independently surveyed physical angle measurements.
Pi build, exact metadata matching rate, latency and stationary live angle
validation remain pending. No navigation was started by this change.

## Horizontal bearing

`TargetObservation.bearing_rad` is the undistorted bbox-centre horizontal angle:
`atan2(x_undistorted, 1)`, in radians, positive image-right, zero along optical
`+z`. It is not robot/map yaw. No elevation angle is published; bbox fields remain
unchanged. Camera motion can still affect this angle: localization must use the
source image timestamp and the actual optical-frame TF.

The default `calibration_file` is `results/updated_calibration/calibration.npz`
when running from the source tree, or the installed package's
`share/ee3033_search_policy/calibration/calibration.npz`. This is the raw
640x480 calibration matching `results/updated_calibration/camera_info.yaml`,
not the alpha0 rectified calibration. Missing/invalid calibration fails startup.
CameraInfo must match its resolution, K, D and plumb_bob model; cropped/binned
geometry is rejected. Mismatches log an error, invalidate the episode and block
observations. `geometry_verified` still requires operator verification that the
detector uses raw pixels and the stated optical frame.

Inverse distortion is checked by forward projection (normalized residual at most
1e-7). Nonconvergent pixels produce a rejection log and no observation, never a
zero-angle fallback. With the current saved coefficients, the extreme right-top
and right-bottom corners fail this check; valid full-image coverage is NOT claimed.
Validate/tune calibration coverage with held-out raw images before deployment.

Rebuild `ee3033_search_policy_interfaces` and re-source the workspace after this
message change; all observation producers/consumers must use the rebuilt type.
Offline geometry tests: `python3 -m pytest test/test_yolo_bearing.py`.

The node deliberately does not publish /target_found; existing navigation cannot
be triggered by it. Typed mission/localizer consumers are NOT integrated yet.
Do not start full_runtime or enable movement to test this checkpoint.

labels_file must be a JSON object exported from the deployed model, e.g.
{"39":"bottle","41":"cup"} ONLY as a clearly labelled synthetic fixture.
No installed model is loaded and no label table is guessed. Require
geometry_verified:=true only after confirming source image dimensions, optical
frame and raw/rectified CameraInfo agreement. Source heartbeat proves transport,
not correct model identity; deployment must separately verify model/checksum.

Configuration requires WAIT_FOR_TARGET, fresh increasing source frames, matching
CameraInfo optical frame and explicit geometry verification. Config locks per UUID
episode. Confirmation occurs only in SEARCHING. IDLE/MAPPING or backward clock
jump invalidates episode; partial sequences reset on pause. Source silence exposes
SOURCE_STALE; mission-level cancellation on health loss is future integration work.

Smoke-test command (separate unused ROS_DOMAIN_ID, no production nodes):

```bash
cd ~/ee3033_ws/src/ee3033_search_policy
export ROS_DOMAIN_ID=115
/usr/bin/python3 -m ee3033_search_policy.yolo_detection_adapter_node --ros-args \
  -p labels_file:=/absolute/path/to/verified_model_labels.json
```

Expected geometry status is GEOMETRY_UNVERIFIED. This checks startup only; do not
set geometry_verified to bypass missing real geometry evidence on the robot.
Simulated tests must publish CameraInfo plus DetectionArray with matching optical
frames and fresh ROS timestamps and publish WAIT_FOR_TARGET then SEARCHING on the
isolated mission topic. Negative cases: unknown class, low confidence, stale/repeated
frames, invalid bbox, missing geometry, config during search and old episode.

Author validation: syntax compilation only; this environment lacks ROS and pytest.
Run test/test_yolo_adapter_session.py then full test suite on Ubuntu. ROS live
startup/service tests are still required, not claimed passed.

Next checkpoint: typed localizer, mission coordinator and verified stop handshake;
then launch/dependencies, automated ROS integration fixtures and robot AI runbook.
Current source package is ament_python: keep interface package as sibling symlink
for colcon and tracked source in interface_packages. Never copy build/install to git.
