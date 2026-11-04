# YOLO Phase 1: offline detection core

Status: implementation only; ROS wiring, navigation and hardware validation pending.

## Added files

- `ee3033_search_policy/yolo_detection_core.py`
- `test/test_yolo_detection_core.py`
- this handoff

No existing ROS node, launch file, vendor package or navigation control is changed.

## Algorithms and verified source schema

User-provided `yolo_msgs/DetectionArray` has a source-image header and detections.
Each detection has class_id, class_name, score, id, bbox.center.position.x/y,
bbox.center.theta and bbox.size.x/y. The core parses these fields without importing
ROS, so it can be tested independently.

`TargetConfiguration` validates a model-derived label table, category, confidence,
WAIT_FOR_TARGET state and detector readiness. Configuration locks until safe reset.
The caller must provide real readiness/state; this class does not discover them.

`select_observation` filters class name AND ID, finite confidence, image bounds,
positive dimensions, valid source header and axis-aligned boxes. Highest confidence
wins with deterministic tie-breaking. Dimensions must match the original detector
image. Class tables must come from the loaded model; test IDs are fixtures only.

`FrameConfirmation` requires increasing distinct source timestamps, freshness,
bounded frame gaps, matching optical frame/category/tracking ID when present, and
bounding-box intersection-over-union continuity. Three observations latch a result
by default. Misses reset a partial sequence. A timer must call tick(now) during
silence. Both now and source timestamp must use the same ROS clock domain; cross-PC
clock synchronization must be checked. Reset explicitly on clock jumps/new mission.

Default values are provisional: confidence .65, 3 frames, age .5 s, gap .4 s,
IoU .2. Not hardware-calibrated. Near image borders or fast motion can cause false
negatives; tune with measured detections, not by removing safety checks.

## Tests and limitations

The author environment lacks pytest in both available Python interpreters. No
pytest pass is claimed. Run the new tests and complete regression suite locally.
This core has no subscriber, publisher, service, or navigation action client.
It DOES NOT implement `/target/set_class`, modify `/target_found`, stop the robot,
or integrate localization. Never start motion to test this phase.

## Next phase after local acceptance and phase commit

1. Confirm team detector is actually this yolo_msgs producer; ee3033_vision may differ.
2. Obtain deployed label table, actual detection topic/QoS and image dimensions.
3. Add ROS adapter and target configuration service/interface package.
4. Connect new checker and localizer to stamped normalized observations.
5. Add explicit detection/stopping/localizing/reporting states with episode IDs.
6. Test no-motion startup and negative cases, then separately authorize physical tests.

Calibration provided is raw 640x480 plumb_bob calibration. Do not use its K with
alpha0-rectified pixels. Localization must undistort raw detection pixels or use
matching rectified image calibration; this phase does not change localization.
