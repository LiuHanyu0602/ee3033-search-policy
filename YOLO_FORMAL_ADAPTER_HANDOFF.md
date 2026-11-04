# Formal YOLO adapter: phase checkpoint

Adds yolo_detection_adapter_node.py, yolo_adapter_session.py and tests.
Run using python -m from repository root; setup.py/launch installation is pending.
Depends on yolo_msgs, rclpy, sensor_msgs, std_msgs, std_srvs and the already-built
ee3033_search_policy_interfaces package. No Ultralytics needed.

Services: /target/set_class (SetTargetClass), /target/reset (Trigger).
Outputs: /target/observation (TargetObservation, volatile),
/target/configuration (latched String JSON), /target/available_classes (latched
String JSON), /target/adapter_status (latched String).
Inputs: parameter detections_topic (default /yolo/detections, must verify live),
camera_info_topic (default /camera_info), mission_topic (default /mission/state).

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
