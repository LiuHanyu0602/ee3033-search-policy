# EE3033 YOLO Target Search and Mission State-Machine Implementation Plan

**Purpose:** This document is the implementation handoff for the next development session. It tells human contributors and coding agents what must be added, what must not be assumed, how the mission states should behave, and how completion will be tested.

**Repository:** `LiuHanyu0602/ee3033-search-policy`
**Development branch at the time of planning:** `robot-validation-hardening`
**Primary deployment environment:** Raspberry Pi ROS 2 Humble Docker workspace at `/home/ros2_ws`
**Safety rule:** Implement and verify the complete perception/state logic with `navigation_enabled:=false` before allowing physical navigation.

---

## 1. Objective

The target must not be hard-coded as `bottle`. At assessment time, the operator must be able to specify the object category announced by the examiner, for example:

- `bottle`
- `cup`
- `backpack`
- another class supported by the deployed YOLO model

After the target is configured, the system shall:

1. validate that the requested class exists in the loaded YOLO label set;
2. lock the target configuration for the current mission;
3. search using the learned Tiny MLP viewpoint-selection policy;
4. accept only fresh, multi-frame YOLO detections of the configured class;
5. stop issuing navigation goals and cancel any active goal;
6. verify that navigation reached a terminal state and the robot stopped;
7. estimate the target position in the `map` frame;
8. publish the coordinate, confidence, and localization method;
9. finish or return home according to mission configuration.

Unknown class names, stale detections, a single accidental detection, or an unverified navigation stop must never be treated as successful target discovery.

---

## 2. Existing validated functionality that must be preserved

Do not remove or bypass the following existing functions:

- the frozen Tiny MLP weights and NumPy inference path;
- runtime candidate-viewpoint generation;
- coverage memory;
- 12-feature construction and shortlist logic;
- Nav2 path planning and `NavigateToPose` interface;
- bounded physical-validation controls:
  - `navigation_enabled`;
  - `single_goal_mode`;
  - `max_navigation_goals`;
  - `/search/cancel_navigation`;
- operator cancellation and terminal-state checking;
- target-localization core:
  - bounding-box bottom/contact-point projection;
  - camera-ray construction;
  - LiDAR gated range selection;
  - ground-plane intersection;
  - robust multi-view triangulation;
  - robust weighted coordinate fusion;
- mission manager, search manager, target checker/localizer, and return-home framework;
- current unit tests. The last recorded complete offline result was **96 passed**.

The new work is an integration and state-semantics improvement. It must not replace the MLP search policy with a fixed waypoint loop.

---

## 3. First task: inspect the real YOLO interface

The current project must not guess the detector message format.

The development environment previously exposed these executables:

```text
ee3033_vision camera_yolo_test
ee3033_vision yolo_viewer
```

The previous Balance detector interface based on `/detector/target_info` and `similarity` is a reference-image matching workflow. Its `similarity` field is **not** YOLO class confidence and must not be reused as such.

Before implementing the adapter, inspect the actual `ee3033_vision` package and record:

- launch/run command;
- subscribed image topic;
- published detection topic;
- exact ROS message type;
- class-name or class-ID field;
- confidence field;
- bounding-box representation;
- header timestamp and frame ID;
- whether all detections or only one detection are published;
- model name and class-label table;
- detector-ready or heartbeat indication;
- measured output rate and latency.

Useful inspection commands inside the correct ROS 2 environment include:

```bash
ros2 pkg prefix ee3033_vision
ros2 pkg executables ee3033_vision

find /home/ros2_ws/src -maxdepth 4 -type f \
  \( -name '*.py' -o -name '*.cpp' -o -name '*.launch.py' -o -name '*.yaml' \) \
  | grep -Ei 'ee3033_vision|yolo'

ros2 topic list -t | grep -Ei 'yolo|detect|bbox|object'
ros2 node list | grep -Ei 'yolo|detect|vision'
ros2 node info <actual_node_name>
ros2 topic info <actual_detection_topic> --verbose
ros2 interface show <actual_message_type>
```

Add the confirmed findings to this document or `WORKFLOW.md`. Do not proceed with a made-up message schema.

---

## 4. Configurable target-category interface

### 4.1 Recommended service

Add a project-owned service, for example:

```text
srv/SetTargetClass.srv
```

Recommended definition:

```text
string class_name
float32 min_confidence
---
bool success
int32 class_id
string normalized_class_name
string message
```

Recommended service name:

```text
/target/set_class
```

Example call:

```bash
ros2 service call \
  /target/set_class \
  ee3033_search_policy/srv/SetTargetClass \
  "{class_name: 'bottle', min_confidence: 0.65}"
```

### 4.2 Supporting interfaces

Provide the following interfaces:

| Interface | Suggested type | Purpose |
|---|---|---|
| `/target/set_class` | `SetTargetClass` service | Validate and set the requested YOLO category |
| `/target/reset` | `std_srvs/srv/Trigger` | Clear the target configuration outside an active search |
| `/target/current_class` | latched `std_msgs/msg/String` | Publish the normalized class used for this mission |
| `/target/available_classes` | project message/service or documented parameter | Expose the detector's supported label set |
| `/target/configured` | latched `std_msgs/msg/Bool` | State whether a valid target is locked |

### 4.3 Validation rules

`/target/set_class` succeeds only when:

- mission state is `WAIT_FOR_TARGET`;
- detector is ready;
- the normalized name exists in the actual model label table;
- confidence threshold is finite and within the accepted range;
- no search or navigation goal is active.

The service must reject:

- spelling errors such as `botle`;
- an unsupported category;
- an empty category;
- NaN/Inf or out-of-range confidence;
- changes during `SEARCHING`, `PAUSING`, `PAUSED`, target stopping, localization, or return-home states.

The validated target must be immutable until the mission finishes or is safely reset.

### 4.4 Temporary fallback if the custom service blocks progress

For the first no-motion prototype only, the target may be set through ROS parameters before starting the mission:

```yaml
target_class: bottle
target_confidence_threshold: 0.65
required_detection_frames: 3
max_detection_age_s: 0.5
max_frame_gap_s: 0.4
```

This is not the preferred final operator interface. Parameter changes must still be rejected or ignored after the target is locked.

---

## 5. YOLO detection adapter

Implement an adapter owned by this repository rather than coupling the mission manager directly to a detector-specific private message.

Suggested module/node:

```text
ee3033_search_policy/yolo_detection_adapter_node.py
```

The adapter shall convert the real detector output into a stable internal representation containing at least:

```text
header stamp
image frame_id
class_name
class_id
confidence
bbox center_x
bbox center_y
bbox width
bbox height
image width
image height
```

Recommended normalized output topic:

```text
/target/detection_candidate
```

The adapter must:

- filter for the currently configured class;
- preserve the original image timestamp;
- reject invalid or non-finite values;
- reject non-positive/out-of-image bounding boxes;
- not convert appearance similarity into class confidence;
- publish detector readiness/health;
- handle multiple detections deterministically;
- document how it chooses between multiple objects of the same class.

For multiple objects of the requested class, a defensible first rule is highest valid confidence, with deterministic tie-breaking. If object identity must remain stable over several frames, add center/IoU association rather than switching arbitrarily.

---

## 6. Multi-frame target confirmation

A single YOLO frame must not trigger mission completion.

Confirm a target only when all conditions hold:

1. detection category equals the locked target category;
2. confidence is at or above the configured threshold;
3. bounding box is valid;
4. detection timestamp is fresh;
5. observations come from distinct image frames;
6. gaps between accepted frames do not exceed the configured limit;
7. detections are spatially consistent using bounding-box center distance and/or IoU;
8. at least `required_detection_frames` observations pass.

Suggested initial values:

```yaml
target_confidence_threshold: 0.65
required_detection_frames: 3
max_detection_age_s: 0.5
max_frame_gap_s: 0.4
```

These values require physical validation. They are starting values, not claimed final calibration.

Publish target confirmation only once per detection episode. Associate the confirmation with an episode ID and timestamp so that an old target coordinate cannot be reused for a new mission.

---

## 7. Required mission states

The final state machine should distinguish configuration, detection, stopping, localization, reporting, and failure. Do not use one broad `TARGET_FOUND` state for all of these operations.

### 7.1 Main states and entry/exit conditions

| State | Entry condition | Successful exit condition |
|---|---|---|
| `IDLE` | System started or safely reset; no active goal | Explicit mapping/start preparation request |
| `MAPPING` | SLAM and required sensors are ready | Map save succeeds and output is validated |
| `WAIT_FOR_TARGET` | Saved map is readable; localization/Nav2 preparation completed | `/target/set_class` succeeds |
| `TARGET_CONFIGURED` | Class exists, threshold is valid, detector is ready and configuration is locked | Explicit start-search request and complete readiness gate passes |
| `SEARCHING` | Map, TF, Nav2, camera, YOLO, MLP and target configuration are ready | Target confirmed, search exhausted, pause requested, operator cancel, or fault |
| `TARGET_DETECTED` | Required number of fresh, consistent frames confirms the configured class | New navigation dispatch is inhibited and cancel request begins |
| `STOPPING_FOR_TARGET` | Navigation cancel/stop requested | Goal reaches verified terminal state and robot stop is confirmed |
| `LOCALIZING_TARGET` | Robot is stopped; current detection and sensor data are available | Valid current `map` coordinate produced |
| `TARGET_REPORTED` | Coordinate, confidence and method have been published for this episode | Finish directly or begin return home |
| `RETURNING_HOME` | Return enabled, home pose valid, target already reported | Nav2 return succeeds and stop is verified |
| `RETURN_FAILED` | Return goal is rejected, aborted, canceled unexpectedly or times out | Operator intervention/reset |
| `FINISHED` | Target reported and mission ended, return succeeded, or search exhausted without detection | Safe reset to `IDLE` |
| `PAUSING` | Pause requested during active operation | Goal cancellation terminal state and stop verified |
| `PAUSED` | Robot confirmed stopped | Explicit resume plus readiness recheck |
| `ERROR_SAFE_STOP` | Critical stale/missing data, invalid state, unverified stop or unrecoverable failure | Operator resolves fault and performs safe reset |

### 7.2 Required transition rules

```text
IDLE
  -> MAPPING
  -> WAIT_FOR_TARGET
  -> TARGET_CONFIGURED
  -> SEARCHING

SEARCHING
  -> TARGET_DETECTED
  -> STOPPING_FOR_TARGET
  -> LOCALIZING_TARGET
  -> TARGET_REPORTED
  -> FINISHED

TARGET_REPORTED
  -> RETURNING_HOME
  -> FINISHED

SEARCHING
  -> PAUSING
  -> PAUSED
  -> SEARCHING

SEARCHING
  -> FINISHED          (coverage/no-gain/budget exhausted without target)

Any active state
  -> ERROR_SAFE_STOP   (critical health or safety failure)
```

### 7.3 Important semantic corrections

- `/mission/start_mapping` must not claim mapping has started unless the mapping stack is actually ready or the external mapping stage is explicitly acknowledged.
- `/mission/finish_mapping` must not merely change a string state. It should verify that the map was saved, files are valid, and the localization stack is ready for the saved map.
- `/mission/start_search` must be rejected unless a valid target is configured and all readiness checks pass.
- Target detection must immediately inhibit new goal dispatches before asynchronous cancellation starts.
- `PAUSED` must not be published before navigation cancellation reaches a known terminal state.
- `/mission/finish` must not jump directly to `FINISHED` while navigation may still be moving.
- Localization must use detection data from the current target episode, not a stale `/target_position` message.
- Return home must not begin until the target coordinate has been successfully reported.
- Unknown navigation status or an unverified stop must enter `ERROR_SAFE_STOP`, not success.

---

## 8. Readiness gates

### 8.1 Before entering `SEARCHING`

Require all of the following:

- map topic available and map metadata valid;
- correct saved map loaded;
- finite and recent `map -> base_footprint` TF;
- Nav2 planner and navigator active;
- planning and navigation actions available;
- camera image stream alive;
- `CameraInfo` available and calibration valid;
- YOLO node alive and producing fresh output/heartbeat;
- configured class exists in current YOLO model;
- Tiny MLP model checksum and reference validation pass;
- no active navigation goal from a previous mission;
- coverage memory is initialized/reset as requested;
- physical-navigation authorization is explicit.

### 8.2 Before entering `LOCALIZING_TARGET`

Require:

- new navigation dispatch disabled;
- cancel/result terminal state verified, or proof that no active goal exists;
- robot motion stopped according to available odometry/velocity evidence;
- detection timestamp still within allowed age;
- associated camera calibration and TF available at the measurement time;
- LiDAR/ground-projection inputs valid for the selected localization method.

### 8.3 Before publishing `TARGET_REPORTED`

Require:

- finite `x`, `y` coordinate;
- frame ID exactly `map`;
- current mission/detection episode match;
- localization timestamp and source recorded;
- quality/confidence metric available;
- localization method identified, e.g. `lidar_gated`, `ground_projection`, `multiview_triangulation`, or `robust_fusion`;
- result is not an old latched coordinate.

---

## 9. Search completion and fault conditions

### 9.1 Normal no-target completion

The system may finish without finding the target only when a documented condition is reached, for example:

- observable coverage target reached;
- no remaining candidate with useful gain;
- no reachable candidate remains;
- maximum navigation-goal budget reached;
- mission timeout reached.

The result must distinguish `TARGET_REPORTED` from `SEARCH_EXHAUSTED_NO_TARGET`.

### 9.2 Conditions that must enter `ERROR_SAFE_STOP`

- map/TF becomes stale or inconsistent;
- robot pose is non-finite or outside the usable map;
- camera/YOLO stream disappears during search;
- configured detector model changes and no longer contains the target class;
- navigation cancel result cannot be verified;
- the robot appears to continue moving after stop timeout;
- target localization produces invalid geometry or non-finite output;
- mission state and active navigation state contradict each other;
- an uncaught node exception or critical dependency failure occurs.

In `ERROR_SAFE_STOP`:

- do not send new navigation goals;
- request cancellation of any active goal;
- publish the fault reason;
- require explicit operator recovery/reset.

---

## 10. Suggested code changes

The implementing agent should inspect current code first and adapt names to the existing package structure. Likely additions/changes are:

```text
srv/SetTargetClass.srv
ee3033_search_policy/yolo_detection_adapter_node.py
ee3033_search_policy/target_checker.py
ee3033_search_policy/mission_manager_node.py
ee3033_search_policy/search_manager_node.py
ee3033_search_policy/target_localizer_node.py
config/search_policy.yaml
launch/full_runtime.launch.py
setup.py
package.xml
README.md
WORKFLOW.md
AGENTS.md
test/test_yolo_detection_adapter.py
test/test_target_configuration.py
test/test_mission_state_machine.py
```

If adding a custom ROS service in the existing Python package is awkward, create a small dedicated interface package such as `ee3033_search_policy_interfaces`. Do not invent a fake copy of the robot's vendor interfaces.

Update launch arguments so the detector topic and message adapter configuration are not hard-coded to one computer.

---

## 11. Required tests

### 11.1 Offline unit tests

Add tests for:

- valid class accepted;
- unknown/misspelled class rejected;
- confidence validation;
- target change rejected outside `WAIT_FOR_TARGET`;
- target configuration locked during search;
- target reset behavior;
- correct category filtering;
- stale detection rejection;
- repeated message/timestamp not counted as multiple frames;
- insufficient consecutive frames rejected;
- inconsistent bounding boxes reset confirmation;
- correct N-frame confirmation;
- new-goal inhibition occurs before navigation cancellation;
- pause enters `PAUSING` before `PAUSED`;
- finish waits for navigation terminal state;
- stale target coordinate rejected;
- return begins only after target report;
- error/safe-stop behavior;
- state transitions reject invalid order.

All pre-existing tests must continue to pass.

### 11.2 No-motion ROS integration test

Run with:

```text
navigation_enabled:=false
```

Acceptance sequence:

1. nodes start without exceptions;
2. state begins at `IDLE`;
3. map/preparation reaches `WAIT_FOR_TARGET`;
4. invalid class is rejected;
5. valid class moves state to `TARGET_CONFIGURED`;
6. start-search reaches `SEARCHING`;
7. wrong-class YOLO detections are ignored;
8. one correct frame is ignored;
9. N fresh consistent frames produce `TARGET_DETECTED`;
10. no navigation command is sent;
11. localization either reports a valid coordinate or a clear, bounded failure;
12. state transition and reason topics match the expected sequence.

### 11.3 Physical staged tests

Do not begin with unrestricted autonomous search.

1. **Detector-only test:** robot stationary, target in view, verify class/confidence/bbox/timestamps.
2. **Single-goal search test:** `single_goal_mode:=true`, `max_navigation_goals:=1`.
3. **Detection while moving:** present target during motion; verify goal cancellation terminal state and physical stop.
4. **Coordinate test:** measure target ground truth and compare reported `map` position.
5. **Multi-goal bounded test:** increase goal budget gradually.
6. **Return-home test:** enable only after reporting and stop logic are validated.

An operator must remain beside the robot with a working emergency stop/cancel mechanism during all motion tests.

---

## 12. Definition of done for the next development session

The YOLO/state-machine integration is considered complete for the first end-to-end milestone only if:

- target category is configurable at runtime before search;
- the category is validated against the actual deployed YOLO model;
- the detector adapter uses real class confidence and real timestamps;
- multi-frame confirmation is implemented and tested;
- `TARGET_CONFIGURED`, `TARGET_DETECTED`, `STOPPING_FOR_TARGET`, `LOCALIZING_TARGET`, and `TARGET_REPORTED` semantics exist;
- pause and finish do not report a stopped/completed state prematurely;
- no-motion end-to-end flow passes;
- all existing and new unit tests pass;
- README/WORKFLOW/AGENTS documentation matches the implemented interfaces;
- all remaining hardware-dependent assumptions are explicitly documented rather than silently guessed.

Physical autonomous-search completion, coordinate-accuracy calibration, and return-home validation may remain separate later milestones if laboratory time is insufficient.

---

## 13. Information still required from the robot environment

Record these values during the next connection to the robot:

```text
ee3033_vision source path:
detector node name:
detector launch/run command:
image input topic:
detection output topic:
detection message type:
class field:
confidence field:
bbox fields:
header timestamp present (yes/no):
model filename:
available class-label source:
CameraInfo topic:
camera optical frame:
LaserScan topic and frame:
map-to-base TF frames:
detector output rate:
typical detector latency:
```

Do not mark these fields complete without command output or source-code evidence.

---

## 14. Short team summary

The assessment target will not necessarily be a bottle. The operator will set the requested YOLO category before the search starts. The system validates and locks that category, then uses the Tiny MLP to choose viewpoints. A target is accepted only after several fresh, consistent YOLO frames. The robot then stops dispatching goals, cancels navigation, verifies that it has stopped, calculates the target's `map` coordinate, publishes the result, and only then finishes or returns home. Missing sensors, stale TF, an unknown class, unverified stopping, or invalid localization must produce a safe failure rather than a false success.
