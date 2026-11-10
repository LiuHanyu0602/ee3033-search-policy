# Formal YOLO adapter: phase checkpoint

Adds yolo_detection_adapter_node.py, yolo_adapter_session.py and tests.
Run using python -m from repository root, or `ros2 run ee3033_search_policy
yolo_detection_adapter` after building the package. Launch integration is pending.
Depends on rclpy, sensor_msgs, std_msgs, std_srvs and the already-built
ee3033_search_policy_interfaces package. Typed input additionally requires
yolo_msgs; NCNN JSON mode does not import it. No Ultralytics needed by the adapter.

Services: /target/set_class (SetTargetClass), /target/reset (Trigger).
Outputs: /target/observation (TargetObservation, BEST_EFFORT/volatile/depth 1),
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

JSON has no frame ID or image size. In JSON mode the adapter uses the matching
`CameraInfo` header and dimensions, retaining only the latest 64 metadata records
and never subscribing to or copying raw image pixels. It matches timestamps
exactly in integer nanoseconds; the output header preserves the detector source
stamp and matching camera frame without substituting newer metadata. Missing,
stale or future source metadata, malformed JSON and invalid bbox values log
explicit rejection and produce no observation. If a detection arrives before
its matching CameraInfo, it is rejected rather than guessed or queued; later
fresh matching frames recover normally.

The JSON subscription is RELIABLE depth 1. Empty detection arrays are valid
heartbeats and break partial confirmation sequences. No tracking ID is invented;
the existing IoU continuity is not proof of persistent physical identity.
CameraInfo geometry must agree with the validated calibration; a change clears
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
This was the offline checkpoint, not a live pass. Subsequent operator-reported
Pi build/tests and stationary live acceptance are recorded below. An exact
metadata matching rate and sustained latency distribution remain unmeasured.
No navigation was started by this change.

## 2026-11-04 stationary Pi acceptance

Classification: **non-actuating robot integration**, with supporting offline
unit tests and a separate C++ timestamp-helper probe. The operator accepted this
stationary phase on 2026-11-04 at approximately 14:19 UTC+08:00. No navigation
goal, localization motion, full runtime or legacy target checker was started
as part of this test.

### Revision, deployment and acceptance boundary

- Last operator-supplied Pi revision:
  `robot-validation-hardening`,
  `9c4a641f50a6ae9d7a316b846371999b7993b2f6`.
  The operator reported interface/package builds and the targeted tests above
  all passed; raw Pi pytest count/timing was not supplied in the acceptance log.
  The 102-test count above is the separately captured FAGIO offline result.
- Pi map deletions/untracked maps were retained. No reset, cleanup of maps,
  model redeployment or modification of the existing detector script occurred.
- Existing detector: `/home/ros2_ws/scripts/yolo_ncnn_node.py`;
  model: `/home/ros2_ws/models/yolov8n_320_ncnn_model`.
  Actual model metadata supplied 80 class labels, including `bottle` ID 39.
  Detector/model checksums were not captured in this phase.
- Raw image: `/camera/image_raw`, 640x480 rgb8; CameraInfo: `/camera_info`,
  frame `default_cam`, matching
  [updated raw calibration](results/updated_calibration/camera_info.yaml).
  Raw calibration uses fx=604.5025053922062, fy=603.4925457708027,
  cx=288.4586293037531, cy=250.20495326829464 and its saved distortion
  coefficients. Do not substitute alpha0 rectified calibration.
- Input JSON is RELIABLE/VOLATILE; image subscription is BEST_EFFORT depth 1.
  Camera target rate was 30 Hz; a measured sustained rate was not archived.
  No map identifier, map pose, optical extrinsics, Nav2 result, coverage,
  inference-only timing, ground-truth target coordinate or position error
  applies to this stationary acceptance.

### Camera timestamp root cause and isolated repair

The original usb_cam 0.8.1 header used:

```cpp
const int64_t epoch_us =
  epoch_time.tv_sec * 1000000 + epoch_time.tv_usec / 1000.0;
```

`tv_usec` is already in microseconds. The erroneous division loses almost all
the fractional second when computing the startup realtime-minus-monotonic
offset, which can create a persistent image timestamp bias. The adapter correctly
rejected these stale source timestamps; its 0.5 s gate was NOT relaxed and
timestamps were NOT replaced with callback receipt time.

The installed-header probe measured errors of 713.453, 883.903, 54.827 and
225.438 ms, tracking the sampled realtime fractional seconds. This directly
proved the helper bug; the later live before/after comparison provided runtime
evidence rather than assuming a header probe alone verified the running binary.

With explicit operator approval, official usb_cam 0.8.1 source
(`cb2ae6bc0a312de629f84fef933fb3e07dac1ef8`) received the
[official single-line fix](https://github.com/ros-drivers/usb_cam/commit/ee0a2f76d8ccf5a4e1e5e88127689d30ba36f243),
replacing `epoch_time.tv_usec / 1000.0` with `epoch_time.tv_usec`.
It was built separately in
`/home/ros2_ws/usb_cam_timestamp_test.hZhIXi`, with a root `COLCON_IGNORE`.
No installed `/opt/ros` files or instructor-provided sources were edited.
The source and build are external Pi artifacts, NOT included in this repository.

Pi ROS Docker, Bash, build terminal K; historical successful build command:

```bash
source /opt/ros/humble/setup.bash
export CAM_WS=/home/ros2_ws/usb_cam_timestamp_test.hZhIXi
cd "$CAM_WS"
colcon build \
  --base-paths "$CAM_WS/src/usb_cam" \
  --packages-select usb_cam \
  --build-base "$CAM_WS/build" \
  --install-base "$CAM_WS/install" \
  --symlink-install --executor sequential \
  --cmake-args -DBUILD_TESTING=OFF
```

Result: one package finished in 21.8 s. The ament header-install-layout
recommendation was not a build failure. This Pi colcon does not support
`--allow-overriding`; no colcon upgrade was performed. Upstream tests were
disabled, not claimed passed. Corrected installed-header probe errors were
0.002, 0.002, 0.001 and 0.001 ms.

After separate approval to switch the camera, the operator stopped only the
original camera and used the isolated overlay with the same device, format,
resolution, rate and calibration. This repair is not activated by pulling
the search-policy repository.

### Observed freshness and bearing

Original raw images were approximately 832-834 ms old; JSON ages observed
approximately 860-1289 ms. After switching, an initial image-only measurement
was 36.59/37.80/38.14 ms (min/median/max). That probe had no JSON samples
because the operator had manually stopped YOLO and the adapter, not because
the new driver was shown to break detection. Both nodes were then restarted.

Final 6 s probe, reporting the most recent 60 callbacks per topic:

| Stream | Samples retained | Min age (ms) | Median age (ms) | Max age (ms) |
| --- | ---: | ---: | ---: | ---: |
| IMAGE | 60 | 37.93 | 39.04 | 40.62 |
| JSON | 60 | 58.36 | 59.11 | 79.01 |

Age is subscriber ROS time minus the original source image timestamp. These
samples passed the unchanged 500 ms gate. They are not a long-run bound or a
measurement of exposure-to-result delay. Subtracting these independent stream
medians does not measure same-frame inference time. The probe did not measure
the adapter's exact metadata cache hit rate.

Captured formal observations:

| Field | Right-side sample | Left-side sample |
| --- | --- | --- |
| Source stamp (sec, nanosec) | 1793773014, 930845000 | 1793773075, 267993000 |
| Frame | default_cam | default_cam |
| Class | 39 / bottle | 39 / bottle |
| Confidence | 0.77587890625 | 0.8115234375 |
| Bbox centre (px) | 432.0, 176.5 | 151.25, 192.75 |
| Bbox width, height (px) | 105.25, 259.25 | 115.5, 273.0 |
| Image size | 640x480 | 640x480 |
| bearing_rad | +0.24120757292339134 | -0.23001782161537115 |
| Approximate degrees | +13.82 | -13.18 |
| confirmed | true | true |
| tracking_id | empty | empty |

Both pasted samples contain episode
`32f8ea72-54c4-4234-b823-ed7a6a1bf21c`. The operator subsequently confirmed
that a new episode and reconfirmation had also been checked and accepted;
the separate service response/new UUID was not pasted or archived. Distinguish
that operator-reported check from the two directly captured observations:
these two samples alone do not prove rearming, since confirmation latches.

Accepted: existing NCNN JSON transport through the calibrated formal adapter,
short-window freshness, left/right bearing signs, and operator-reported target
configuration/reconfirmation. Not accepted by this evidence: surveyed absolute
angle accuracy, a near-zero sample, full-image calibration coverage, persistent
same-instance association, camera-to-map TF, search cancellation, active
multi-view localization, target map coordinates or approach navigation.

### Reusing the accepted stationary setup

All commands below are **Pi ROS Docker, Bash**. Enter the existing ROS container
from a separate Pi host terminal using `ros2-start`. Do not automatically start
the robot stack or run these commands alongside duplicate camera/detector nodes.

Camera terminal H, only after approval to stop the old camera and verification
that `/dev/video0` is free:

```bash
source /opt/ros/humble/setup.bash
source /home/ros2_ws/install/setup.bash
export CAM_WS=/home/ros2_ws/usb_cam_timestamp_test.hZhIXi
source "$CAM_WS/install/local_setup.bash"
ros2 pkg prefix usb_cam
```

Require prefix `/home/ros2_ws/usb_cam_timestamp_test.hZhIXi/install/usb_cam`;
stop if it differs or the external workspace is missing. Then:

```bash
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -r __node:=usb_cam \
  -p video_device:=/dev/video0 \
  -p framerate:=30.0 \
  -p image_width:=640 \
  -p image_height:=480 \
  -p pixel_format:=mjpeg2rgb \
  -p camera_name:=ee3033_camera \
  -p camera_info_url:=file:///home/ros2_ws/src/ee3033-search-policy/results/updated_calibration/camera_info.yaml \
  -r /image_raw:=/camera/image_raw
```

YOLO terminal Y:

```bash
source /opt/ros/humble/setup.bash
source /home/ros2_ws/install/setup.bash
python3 /home/ros2_ws/scripts/yolo_ncnn_node.py
```

Adapter terminal J; the labels file below is the actual session export, not a
permanent installation. If absent, re-export the actual model's metadata labels;
do not substitute an empty file or guessed class table.

```bash
source /opt/ros/humble/setup.bash
source /home/ros2_ws/install/setup.bash
ls -l /tmp/ee3033_yolo_labels_520cpu0n.json
```

Only if the verified labels file exists:

```bash
ros2 run ee3033_search_policy yolo_detection_adapter --ros-args \
  -p input_format:=ncnn_json \
  -p labels_file:=/tmp/ee3033_yolo_labels_520cpu0n.json \
  -p detections_topic:=/yolo/detections \
  -p camera_info_topic:=/camera_info \
  -p mission_topic:=/bearing_test/mission_state \
  -p geometry_verified:=true
```

Control terminal D, after checking fresh frames and that the adapter's
`mission_topic` is `/bearing_test/mission_state`:

```bash
source /opt/ros/humble/setup.bash
source /home/ros2_ws/install/setup.bash
ros2 param get /yolo_detection_adapter mission_topic
ros2 topic pub --once --qos-durability transient_local \
  /bearing_test/mission_state std_msgs/msg/String "{data: WAIT_FOR_TARGET}"
ros2 service call /target/set_class \
  ee3033_search_policy_interfaces/srv/SetTargetClass \
  "{class_name: bottle, min_confidence: 0.65}"
```

Require success, class ID 39 and a nonempty new episode before continuing:

```bash
ros2 topic pub --once --qos-durability transient_local \
  /bearing_test/mission_state std_msgs/msg/String "{data: SEARCHING}"
timeout 10s ros2 topic echo /target/observation \
  ee3033_search_policy_interfaces/msg/TargetObservation \
  --once --full-length --filter 'm.confirmed' \
  --qos-reliability reliable --qos-durability volatile
```

This state is an isolated adapter input, NOT authorization for robot search.
Reconfigure and record a new episode when testing independent confirmation;
do not interpret a latched `confirmed=true` as fresh three-frame confirmation.

Rollback, if separately authorized: Ctrl+C only the corrected camera, then use
a **clean Docker Bash terminal** sourcing `/opt/ros/humble/setup.bash` and
`/home/ros2_ws/install/setup.bash` without the camera overlay. Verify
`ros2 pkg prefix usb_cam` returns `/opt/ros/humble`, then reuse the same camera
command above. This restores the original driver, including its timestamp bug;
keep adapter freshness rejection in place. Do not delete the isolated workspace
or overwrite system files to roll back.

Next work requires a separately approved phase: inspect actual TF/pose and typed
localizer/mission interfaces, then establish a verified navigation-stop handshake
before any deliberate localization motion. No active localization or approach
command is authorized by this acceptance.

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

Original adapter-author validation was syntax compilation only. Subsequent
FAGIO targeted tests and Pi stationary acceptance are recorded above; do not
interpret these as a full regression suite or mission/localizer integration pass.

Next checkpoint: typed localizer, mission coordinator and verified stop handshake;
then launch/dependencies, automated ROS integration fixtures and robot AI runbook.
Current source package is ament_python: the genuine interface package is tracked
in interface_packages. Pi successfully built it with explicit `--base-paths`;
a sibling symlink is optional, not required. Never copy build/install to git.
