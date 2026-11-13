# EE3033 Learned Target-Search Policy

This repository contains the offline training and evaluation pipeline for a
lightweight learned viewpoint-selection policy for an EE3033 mobile-robot
target-search task.

The robot first constructs a 2-D occupancy map. After the target is placed,
the policy scores candidate camera viewpoints, Nav2 drives the robot to the
selected pose, and the searched-area memory is updated. Object detection and
target-coordinate estimation are separate downstream modules.

## Current status

- Synthetic maze generation: complete
- Safety-mask and candidate-viewpoint generation: complete
- Two-step heuristic teacher and expert-dataset generation: complete
- Tiny MLP training and held-out-map evaluation: complete
- NumPy model export: complete
- Raspberry Pi ROS 2 integration: in progress

The policy network has architecture `12 -> 32 -> 16 -> 1` with 961 trainable
parameters. It assigns one scalar score to each candidate viewpoint. The
highest-scoring reachable candidate is sent to the navigation layer.

## Held-out evaluation

The current V2 model was trained using 30 generated maze maps. Six maps were
held out for testing. Multi-map evaluation used three shared initial poses per
test map, giving 18 paired rollout trials.

| Policy | Coverage | Expected detection time | Total time | Path length |
| --- | ---: | ---: | ---: | ---: |
| Coverage only | 99.60% | 428.84 s | 1434.64 s | 339.54 m |
| Immediate time-aware | 99.56% | 125.91 s | 481.22 s | 100.81 m |
| Two-step expert | 99.60% | 103.71 s | 402.07 s | 83.24 m |
| Tiny MLP | 99.58% | 101.04 s | 370.93 s | 73.30 m |

Compared with the immediate time-aware baseline, the Tiny MLP reduced expected
detection time by 19.75% and path length by 27.29%, while maintaining 99.58%
observable-space coverage. Its rollout performance was comparable to the more
computationally expensive two-step heuristic teacher.

These are simulation results. They do not yet include localization drift,
Nav2 failures, camera occlusion, detector errors, or physical robot dynamics.

## Repository layout

```text
.
├── scripts/                 Offline generation, training and evaluation
├── maps/                    Small reference map used during development
├── models/                  Frozen deployment weights
├── results/v2/              Selected reproducible result files
├── MODEL_CARD.md            Model inputs, metrics and limitations
├── requirements-training.txt
└── requirements-runtime.txt
```

Generated datasets and map intermediates under `outputs/` are intentionally
excluded from Git. They can be regenerated from the scripts and recorded
configuration.

## Reproduce the V2 pipeline

Run all commands with Ubuntu system Python unless a different environment is
explicitly required.

### 1. Generate maps

```bash
/usr/bin/python3 scripts/generate_training_maps_v2.py \
  --count 30 \
  --base-seed 13033 \
  --name-prefix general_maze \
  --start-index 0 \
  --output-root outputs/training_maps_v2
```

### 2. Generate teacher decisions

```bash
/usr/bin/python3 scripts/generate_expert_dataset.py \
  --root outputs/training_maps_v2 \
  --episodes-per-map 3 \
  --max-steps 120 \
  --coverage-target 0.995 \
  --shortlist-size 32 \
  --successor-count 16 \
  --linear-speed 0.25 \
  --angular-speed 0.8 \
  --observation-time 0.25 \
  --time-discount 20.0 \
  --seed 23033
```

### 3. Train the Tiny MLP

```bash
/usr/bin/python3 scripts/train_tiny_mlp.py \
  --dataset outputs/training_maps_v2/expert_dataset.npz \
  --epochs 300 \
  --batch-size 64 \
  --learning-rate 0.001 \
  --weight-decay 0.0001 \
  --teacher-temperature 0.15 \
  --mse-weight 0.2 \
  --patience 40 \
  --seed 33033
```

### 4. Evaluate held-out maps

```bash
/usr/bin/python3 scripts/evaluate_policy_rollouts.py \
  --training-root outputs/training_maps_v2 \
  --episodes 3 \
  --seed 44033
```

## Frozen V2 deployment model

```text
models/tiny_search_policy_weights.npz
SHA256: 3f2c9e784f71d3d5921f8e699d0013ca377034ff70d6f87722b73062d132a3b5
```

The `.npz` file includes feature normalization statistics and all three layer
weights and biases. PyTorch is not required for NumPy inference.

See `MODEL_CARD.md` before using the model because feature order and units must
match training exactly.

## Non-actuating Nav2 planning test (ROS 2 Humble)

The standalone `planner_test` executable selects two safe points from `/map`,
requests `/compute_path_to_pose` with an explicit start and `use_start=True`,
and publishes diagnostics for RViz. It does **not** use the robot's current pose
as its start, send navigation goals, publish velocity commands, launch Nav2,
or run a waypoint loop. Motors may be disconnected for this test.
The existing `search_policy_dry_run` executable and launch remain unchanged.

### Build and run inside the robot's ROS Docker environment

The host development machine does not need ROS installed. Run ROS commands
inside the ROS-enabled container, with this repository under the workspace's
`src` directory. Nav2 and the map server must already be running and active.

```bash
source /opt/ros/humble/setup.bash
cd /home/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select ee3033_search_policy --symlink-install
source install/setup.bash
ros2 launch ee3033_search_policy planner_test.launch.py
```

One request is made per launch. Restart the test node to run another test;
map updates never trigger an automatic retry. The node stays alive afterward
to retain visualization data until Ctrl+C.

Expected successful output (values depend on the map and Nav2 configuration):

```text
Planning SUCCEEDED: status=4, poses=<N>, length=<metres> m, planning_time=<ms> ms, request_latency=<ms> ms.
```

`planning_time` is Nav2's reported computation time; `request_latency` measures
the complete local Action round trip, including goal acknowledgement. Neither
is estimated robot travel time. Goal acceptance alone is not success: aborted,
canceled, empty, non-finite, or inconsistent-frame paths fail explicitly.
Local errors/timeouts without a terminal server result use Action status `0`
(UNKNOWN). A timeout requests best-effort cancellation; it does not confirm
that the server stopped processing.

### Test-point configuration

Defaults are in [config/planner_test.yaml](config/planner_test.yaml).
Use a separate edited YAML file without changing readiness-node settings:

```bash
ros2 launch ee3033_search_policy planner_test.launch.py \
  params_file:=/absolute/path/to/planner_test.yaml
```

| Parameter | Default | Purpose |
| --- | --- | --- |
| `map_topic` | `/map` | Transient-local occupancy map |
| `planner_action` | `/compute_path_to_pose` | Planning Action name |
| `planner_id` | `GridBased` | Nav2 planner plugin ID |
| `clearance_m` | `0.20` | Conservative cell-centre clearance |
| `min_separation_m` | `1.0` | Minimum selected-point separation |
| `free_threshold` | `20` | Maximum known-free occupancy value |
| `occupied_threshold` | `65` | Minimum occupied value |
| `readiness_timeout_s` | `30.0` | Wait limit for map and planner |
| `request_timeout_s` | `15.0` | Planning Action round-trip deadline |

Occupancy values `-1` and intermediate values between the thresholds are
excluded from free space. The mask clears obstacles, unknown/ambiguous cells,
and map boundaries. It conservatively accounts for blocked cell area, so it
can exclude more cells than an exact footprint calculation.
Both points come from the largest four-connected safe-space component.
Farthest-point sweeps select a separated pair deterministically; they do not
compute an exact component diameter. If the chosen pair is too close or no
valid component exists, the test logs an error without submitting a goal.

**The default clearance is not a verified physical robot footprint.** Configure
it for the intended test; this mask does not replace Nav2's footprint,
costmaps, obstacle inflation, or collision-aware planning. Shared mask
connectivity does not guarantee Nav2 success.

Coordinates use cell centres, the map's actual dimensions/resolution, and the
origin quaternion's planar yaw. OccupancyGrid rows are not vertically flipped
like saved PGM images. Invalid/nonplanar origins are rejected. The map's
`header.frame_id` is used for poses; no TF lookup or hardcoded map geometry
is required for this explicit-start test.

For simulation, launch with `use_sim_time:=true`. Map/planner readiness and
Action deadlines use steady time, so a stopped `/clock` cannot prevent timeout.

### RViz, including a separate Ubuntu computer

Keep `planner_test` running. On the RViz computer, source ROS 2 and use the
same `ROS_DOMAIN_ID` and compatible DDS settings as the robot. Ensure discovery
and DDS traffic can pass between the computer and the Docker container.
`ROS_LOCALHOST_ONLY` must not restrict communication to localhost. Container
networking/firewall setup must be checked separately; matching domain IDs alone
does not establish connectivity.

Start `rviz2` manually; the launch file never starts it on the Raspberry Pi.
Set **Fixed Frame** to the map's frame (usually `map`) and add:

| Display | Topic | Message type |
| --- | --- | --- |
| Map | `/map` | `nav_msgs/msg/OccupancyGrid` |
| Pose | `/search/debug/start` | `geometry_msgs/msg/PoseStamped` |
| Pose | `/search/debug/goal` | `geometry_msgs/msg/PoseStamped` |
| Path | `/search/debug/path` | `nav_msgs/msg/Path` |
| MarkerArray | `/search/debug/markers` | `visualization_msgs/msg/MarkerArray` |

Set display subscriptions to **Reliable**, **Transient Local**, depth **1**
where available. All four debug publishers retain their latest message while
the node is alive, allowing RViz to start after planning completes. Green and
orange arrows plus START/GOAL labels identify the selected points. A path is
published only for a validated successful result.

Check retained output from a sourced ROS terminal:

```bash
ros2 topic echo /search/debug/path --once \
  --qos-reliability reliable --qos-durability transient_local
ros2 node info /planner_test
```

The node should list only the planning Action client, not a navigation client,
and no velocity publisher. Inspect node endpoints, not the global absence of
`/cmd_vel`, since other robot nodes may legitimately expose that topic.
Do not use RViz's navigation-goal tools as part of this non-actuating test.

### Reusing the client

[nav2_client.py](ee3033_search_policy/nav2_client.py) exposes `Nav2Client`,
`ActionRequest`, `PathResult`, and the separate `NavigationResult`.
For an existing spinning node:

```python
from ee3033_search_policy.nav2_client import Nav2Client

nav2 = Nav2Client(node)
# Poll nav2.planner_is_ready() from a timer; do not block a callback.
request = nav2.compute_path(start_pose, goal_pose, timeout_s=15.0)
request.future.add_done_callback(on_path_result)
# Inside on_path_result: result = future.result(); inspect result.success.
# If needed: cancel_ack = request.cancel()  # asynchronous Future[bool]
# Before destroying the owning node: nav2.close()
```

Each Action channel accepts one active local request. An overlapping request
returns an explicitly failed result and is not sent to Nav2. Sequential calls
are supported. A local timeout completes the result and frees the slot;
late acceptance triggers cancellation of that specific old goal, and late
results cannot replace another request's result. If the server never answers,
cancellation acknowledgement may remain pending. Do not block/spin recursively
inside callbacks or call `request.future.cancel()`; use `request.cancel()`.
Cancellation acknowledgement is not terminal completion: inspect the request's
result for its actual terminal Action status.

`navigator_is_ready()` lazily creates a navigation client for discovery only;
it does not enable goal submission. Navigation requires both constructor
`enable_navigation=True` and an explicit `navigate_to_pose(...)` call.
That method accepts a feedback callback receiving the ROS feedback envelope,
returns a separate request/result, and supports asynchronous cancellation.
The planner test never enables navigation or calls that method.
Never replace a timed-out navigation goal without first checking robot state:
local timeout/cancellation is not proof that navigation has stopped.
**Physical navigation submission, feedback, completion, and cancellation remain
unvalidated and are outside this planning-only acceptance test.**

### Tests and deployment acceptance

Pure Python tests do not need ROS. In a development virtual environment with
NumPy and pytest installed:

```bash
python3 -m pip install -r requirements-runtime.txt pytest
python3 -m pytest -q test
python3 -m compileall -q ee3033_search_policy launch test
```

Tests cover rotated map origins, occupancy classification, conservative
clearance, connected point selection, path metrics, and Action lifecycle
behavior with test doubles. These doubles do **not** validate DDS,
Humble message serialization, launch behavior, actual Nav2 planning, or RViz.

Inside the ROS container after building:

```bash
cd /home/ros2_ws
source install/setup.bash
colcon test --packages-select ee3033_search_policy
colcon test-result --verbose
```

Robot acceptance is still required:

1. Re-run `search_policy_dry_run.launch.py` to confirm existing readiness behavior.
2. Launch the planner test with an active map server and planner.
3. Verify status `4`, a nonempty path, and all reported timing/length fields.
4. Start remote RViz afterward and verify the retained path and markers.
5. Inspect `/planner_test` endpoints to confirm no navigation client or velocity
   publisher; keep other controllers and RViz goal tools outside this test.

For failures, check `/map` durability, Nav2 lifecycle activation, planner ID,
the map frame versus Nav2's global frame, and network discovery. For insufficient
safe points, inspect the map and configured clearance/separation rather than
treating unknown cells as free. The test does not silently reduce clearance,
change planner IDs, or retry different goals.

## Camera and detector integration status

The final project requirement is **automatic YOLO category detection of a
`bottle`**. It must not require an operator click or a reference crop.

The currently installed `balance_object_detect` node implements a different
task: reference-image/instance matching. It remains `IDLE` until a target
image or click is supplied, and its public `TargetInfo.similarity` value is
appearance similarity, not YOLO class confidence. The current
`target_checker` consumes that legacy interface. Therefore the installed
reference detector cannot be converted into automatic bottle-class detection
by a launch argument or service command.

The final required path is:

```text
live camera
  -> YOLO class detections
  -> filter class=bottle and confidence threshold
  -> fresh multi-frame confirmation
  -> /target_found
  -> bbox-based target localization
  -> navigation cancellation and verified stop
```

At this revision, the repository has **no adapter for a generic YOLO detection
message**, so the final bottle workflow is not yet implemented. Before writing
that adapter, inspect the actual YOLO node available on the selected compute
machine and record its topic, message type, fields, QoS and image input:

```bash
ros2 pkg list | grep -Ei 'yolo|detect'
ros2 pkg executables | grep -Ei 'yolo|detect'
ros2 topic list -t | grep -Ei 'yolo|detect'
```

For each discovered detection topic:

```bash
ros2 topic info <DETECTION_TOPIC> --verbose
ros2 interface show <DETECTION_MESSAGE_TYPE>
```

Do not invent `/yolo/detections` or assume it is JSON. Do not reinterpret
`TargetInfo.similarity` as class confidence without an explicit adapter and
tests. Navigation must remain disabled while this interface is being added.

The remainder of this section documents the **legacy reference detector only**
for camera/interface diagnostics. It is not the final bottle-search workflow.

All commands below run inside the robot ROS Docker container unless explicitly
labelled otherwise. On the Raspberry Pi host, enter the existing container with:

```bash
docker exec -it ros2_humble bash
```

In every new Docker terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/ros2_ws/install/setup.bash
```

Do not launch the camera or detector twice. Before starting either component,
check for existing instances:

```bash
ros2 node list | grep -E '^/usb_cam$|^/object_detector_node$' || true
pgrep -af 'usb_cam|balance_object_detect|object_detector' || true
```

If one is already running, reuse it. If duplicate nodes exist, stop the extra
launch from the terminal that owns it with Ctrl+C; do not start another copy.

### 1. Start the USB camera

Open Docker terminal A:

```bash
source /opt/ros/humble/setup.bash
source /home/ros2_ws/install/setup.bash

ros2 launch balance_usb_cam usb_cam.launch.py mode:=detector
```

Keep this terminal running. The measured detector-mode interfaces are:

| Name | Type |
| --- | --- |
| `/image_raw` | `sensor_msgs/msg/Image` |
| `/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` |
| `/camera_info` | `sensor_msgs/msg/CameraInfo` |

Verify them from Docker terminal B:

```bash
source /opt/ros/humble/setup.bash
source /home/ros2_ws/install/setup.bash

ros2 node list | grep -x /usb_cam
ros2 topic info /image_raw/compressed --verbose
ros2 topic info /camera_info --verbose
timeout 8 ros2 topic hz /image_raw
timeout 5 ros2 topic echo /camera_info --once
```

Expected: one `/usb_cam` node, one image publisher, approximately 10 Hz in
the measured detector configuration, and a valid `CameraInfo` message.
A timeout is used so diagnostic commands do not block indefinitely.

### 2. Start the Balance object detector

Open Docker terminal C:

```bash
source /opt/ros/humble/setup.bash
source /home/ros2_ws/install/setup.bash

ros2 launch balance_object_detect balance_object_detector.launch.py \
  rqt_view:=False
```

The launch defaults currently select `yolo26n.pt`,
`osnet_x0_25`, and `mobilenet_v3_small`. Keep the detector terminal
running. Verify that exactly one detector is connected to the camera:

```bash
ros2 node list | grep -x /object_detector_node
ros2 node info /object_detector_node
ros2 topic info /image_raw/compressed --verbose
ros2 service list -t | grep '^/detector/'
ros2 topic list -t | grep '^/detector/'
```

The real detector interfaces are:

| Interface | Type | Purpose |
| --- | --- | --- |
| `/detector/set_target` | `balance_interface/srv/SetTarget` | Supply a cropped target reference image |
| `/detector/reset_target` | `std_srvs/srv/Empty` | Clear the detector target |
| `/detector/status` | `std_msgs/msg/String` | `IDLE`, `SEARCHING`, `TRACKING`, etc. |
| `/detector/target_info` | `balance_interface/msg/TargetInfo` | Detection flag, bbox and cosine similarity |
| `/detector/result_image/compressed` | `sensor_msgs/msg/CompressedImage` | Annotated debug image |
| `/detector/click_point` | `geometry_msgs/msg/Point` | Detector GUI support; not required by this CLI workflow |

Confirm the installed definitions rather than assuming them:

```bash
ros2 interface show balance_interface/srv/SetTarget
ros2 interface show balance_interface/msg/TargetInfo
```

Before a target is set, the expected detector status is `IDLE`.

### 3. Optional legacy diagnostic: prepare a reference image

The official CLI client expects an image file that is accessible **inside the
Docker container**. Use a close, sharp, tightly cropped image containing the
target object and little background. Do not use the entire camera frame: the
detector would learn the background together with the object.

One reproducible method is to crop a camera screenshot on a laptop and copy it
to the bind-mounted repository. From the laptop/Ubuntu computer, create the
robot-side directory first and then copy the crop:

```bash
ssh balance@<ROBOT_IP> \
  'mkdir -p /home/balance/ros2_ws/src/ee3033-search-policy/target_samples'

scp bottle_target.jpg \
  balance@<ROBOT_IP>:/home/balance/ros2_ws/src/ee3033-search-policy/target_samples/bottle_target.jpg
```

Replace `<ROBOT_IP>` with the current address, for example
`10.75.60.142` on the active hotspot session.

The same file is then visible inside Docker at:

```text
/home/ros2_ws/src/ee3033-search-policy/target_samples/bottle_target.jpg
```

Check it inside Docker:

```bash
ls -lh /home/ros2_ws/src/ee3033-search-policy/target_samples/bottle_target.jpg
```

Target reference photographs are experiment inputs and should not be committed
unless the team deliberately wants them in Git.

### 4. Optional legacy diagnostic: set an appearance target

This command tests the installed reference-matching detector only. It must not
be described as automatic YOLO bottle classification.

With the camera and detector still running:

```bash
ros2 run balance_object_detect test_object_detect_client \
  /home/ros2_ws/src/ee3033-search-policy/target_samples/bottle_target.jpg
```

Expected response includes `SUCCESS`. The detector status should change from
`IDLE` to `SEARCHING`, and to `TRACKING` when the live camera matches the
reference target.

Monitor status without an endless command:

```bash
timeout 10 ros2 topic echo /detector/status
```

Monitor the real detection payload using its measured BEST_EFFORT QoS:

```bash
timeout 10 ros2 topic echo /detector/target_info \
  --qos-reliability best_effort
```

A useful matched message has `detected: true`, positive bbox width/height,
and a finite similarity. A stream of `detected: false` while status is
`SEARCHING` means the pipeline is running but the live image has not matched
the reference strongly enough.

### 5. Legacy integration test: confirmation and localization

The unified launch starts `target_checker`, `target_localizer`,
`mission_manager`, and `search_manager`. Its safe default does not send
navigation goals:

```bash
ros2 launch ee3033_search_policy full_runtime.launch.py \
  navigation_enabled:=false \
  single_goal_mode:=true \
  max_navigation_goals:=1
```

The real checker consumes:

| Input | Type |
| --- | --- |
| `/detector/status` | `std_msgs/msg/String` |
| `/detector/target_info` | `balance_interface/msg/TargetInfo` |

Default confirmation requires status `TRACKING`, `detected=true`,
similarity at least `0.85`, a positive bbox, and three consecutive valid
frames. It then latches `/target_found=true`.

Inspect the complete chain:

```bash
ros2 node info /target_checker

timeout 10 ros2 topic echo /target_found --once \
  --qos-reliability reliable \
  --qos-durability transient_local

timeout 10 ros2 topic echo /target_position_candidate
timeout 10 ros2 topic echo /target_position --once \
  --qos-reliability reliable \
  --qos-durability transient_local
```

`/target_position` also requires valid `/camera_info`, `/scan`, TF, and
the target-localizer calibration parameters. A confirmed visual detection does
not by itself prove that the reported map coordinate is accurate.

### 6. Reset or change the target

First clear the detector's current reference:

```bash
ros2 service call /detector/reset_target std_srvs/srv/Empty "{}"
```

Then clear this repository's latched confirmation:

```bash
ros2 service call /target_checker/reset std_srvs/srv/Trigger "{}"
```

Confirm the reset:

```bash
timeout 5 ros2 topic echo /detector/status --once

timeout 5 ros2 topic echo /target_found --once \
  --qos-reliability reliable \
  --qos-durability transient_local
```

Expected: detector status `IDLE` and `/target_found.data=false`. To search
for a different object, run `test_object_detect_client` again with its new
tight crop.

### 7. Common failure checks

| Symptom | Check |
| --- | --- |
| Detector remains `IDLE` | The `SetTarget` call did not succeed or its file path is not visible inside Docker |
| No image subscriber | Start one camera and one detector; inspect `/image_raw/compressed --verbose` |
| Two detector publishers/subscribers | Duplicate detector launches; stop the extra launch with Ctrl+C |
| `SEARCHING` with `detected=false` | Poor/incorrect reference crop, target not visible, scale/viewpoint mismatch or insufficient similarity |
| Detector says `TRACKING` but `/target_found=false` | Check similarity, bbox size, three-frame continuity, status freshness, and that `target_checker` is running |
| `/target_found=true` but no coordinate | Check `/camera_info`, `/scan`, `map -> base_footprint`, camera extrinsics and localizer logs |
| Camera launch prints display/xhost errors | Keep `rqt_view:=False`; visualization should run on a computer with a valid display |
| Diagnostic command appears stuck | Use the documented `timeout` wrapper; topic echo/hz normally waits for messages |

When `/target_found=true` arrives during an enabled search, SearchManager
inhibits later goal dispatch and requests cancellation of its owned Nav2 goal.
A cancellation request is not proof of physical standstill; verify the final
Action result and robot motion before declaring a successful stop.


## Full ROS 2 runtime interface (development branch)

The `robot-runtime-integration` branch builds on the return-to-origin and
target-localization development line, which includes the software-facing
Stage-2 loop. It is intentionally non-actuating by default: the search manager publishes
its selected goal but cannot send NavigateToPose unless
`navigation_enabled:=true` and `/mission/state` is `SEARCHING`.

The motion gate is named `navigation_enabled`; `enable_motion` is not a
supported parameter alias. Keep navigation disabled until live deployment
checks and explicit on-site authorization are complete.

Before recording the search home or planning, the robot TF position must be
finite and inside the occupancy map, including its translated/rotated origin.
Otherwise `/search/status` reports `ROBOT_POSE_OUTSIDE_MAP` and logs the
localization error. Missing map, TF, or planner readiness also blocks planning.
A rejected planning goal stops that candidate batch, reports
`WAITING_FOR_PLANNER_ACTIVE`, and waits five seconds before retrying. Rejection
does not blacklist a candidate or prove the planner is inactive; inspect its
actual lifecycle state. Return-home handling and observation completion are
not delayed by this planner backoff.

Offline regression tests use ROS doubles and do not establish TF freshness,
localization health, planner lifecycle readiness, or physical navigation safety.

### Runtime data flow

1. `/map` generates oriented viewpoints only on known-free cells with the
   operator-confirmed 0.15 m clearance; unknown/ambiguous cells are blocked.
2. `map -> base_footprint` restricts the first shortlist to the robot's
   four-connected safe-space component.
3. Coverage memory computes new visible area for every viewpoint.
4. Nav2 `/compute_path_to_pose` supplies real path distances for up to 24
   connected preliminary candidates.
5. Up to 24 path-valid candidates receive the training-consistent 12 features.
6. Tiny MLP selects a viewpoint; invalid inference falls back to the immediate
   time-aware score.
7. Nav2 executes one goal. Coverage is committed only after success and the
   observation dwell time.
8. Navigation failure counts temporarily blacklist repeated unreachable goals.
9. A confirmed target latches `/target_found`, cancels the active goal and
   prevents all later dispatch.
10. RGB bearing plus LaserScan (or calibrated ground-plane fallback) publishes
   the target in the map frame.

### Nodes

| Executable | Role |
| --- | --- |
| `target_checker` | Confirms real Balance detector TRACKING messages over consecutive frames |
| `target_localizer` | Converts image bearing and range to `/target_position` in `map` |
| `mission_manager` | Implements mapping, wait, search, pause and finish states |
| `search_manager` | Runs viewpoint generation, planning, MLP selection, navigation and memory |

### Mission services

- `/mission/start_mapping`
- `/mission/finish_mapping`
- `/mission/start_search`
- `/mission/pause`
- `/mission/resume`
- `/mission/finish`
- `/mission/reset`

All use `std_srvs/srv/Trigger`. The state is latched on
`/mission/state`. `/mission/notification` is the hardware-independent
notification interface; a later Balance-specific LED/buzzer adapter should
subscribe to it.

### Search interfaces

| Name | Type | Purpose |
| --- | --- | --- |
| `/search/status` | `std_msgs/String` | Current planner/navigation state |
| `/search/coverage_fraction` | `std_msgs/Float32` | Remembered observable coverage |
| `/search/selected_goal` | `geometry_msgs/PoseStamped` | Selected viewpoint for RViz |
| `/search/finished` | `std_msgs/Bool` | Coverage/no-gain completion |
| `/search/reset_memory` | `std_srvs/Trigger` | Clear coverage and failure blacklist |
| `/search/save_memory` | `std_srvs/Trigger` | Save memory to configured NPZ |
| `/search/load_memory` | `std_srvs/Trigger` | Restore matching-map memory |

### Target outputs

- `/target_found`: latched confirmed detection
- `/target_position_candidate`: unconfirmed per-frame coordinate estimate
- `/target_position`: latched confirmed `geometry_msgs/PointStamped` in `map`
- `/target_confidence`: detector similarity
- `/target_position_method`: `lidar_association` or `ground_plane`
- `/target_report`: latched human-readable final frame/x/y report

### Unified launch

Safe interface-only mode:

```bash
ros2 launch ee3033_search_policy full_runtime.launch.py
```

Physical navigation is an explicit later opt-in:

```bash
ros2 launch ee3033_search_policy full_runtime.launch.py \
  navigation_enabled:=true
```

Do not enable navigation until the branch has been built and validated with
the robot raised or in a controlled test area.

### Parameters that still require physical calibration

Software interfaces are present, but these values cannot be inferred safely
without the robot:

- future changes to `candidate_clearance_m=0.15` versus the verified robot
  geometry and Nav2's default inflation configuration;
- `camera_pitch_down_rad` and camera optical-axis convention;
- `camera_yaw_offset_rad` and `laser_yaw_offset_rad`;
- whether the 2-D laser intersects the target rather than the wall behind it;
- camera FOV/range and navigation speed/time assumptions;
- the robot-specific LED or buzzer command interface.

Until calibration, `target_position_method` must be logged with every reported
coordinate and the target coordinate must be treated as provisional.


## Return to origin (`return-to-origin-integration`)

After a confirmed target, the search manager cancels its active Nav2 search goal. A home pose is saved in the `map` frame at the beginning of the search (at first available valid TF). If `navigation_enabled:=true` and `return_home_enabled:=true`, the controller waits for a verified search navigation result (`CANCELED` or `SUCCEEDED`, accepted and not locally timed out) before sending a new `NavigateToPose` goal back to the saved start pose. If no search goal was active, it can return directly. An unknown/timeout/cancellation failure prevents automatic return; an Action cancellation acknowledgement alone is not enough. It reuses the **same** Nav2 client, preventing competing action owners inside this package.

`/search/return_status` (`std_msgs/msg/String`, reliable/transient-local) reports `HOME_RECORDED`, `WAITING_FOR_SEARCH_STOP`, `WAITING_FOR_NAVIGATOR`, `RETURNING`, `SUCCEEDED`, or `FAILED_*`. Mission states include `TARGET_FOUND -> RETURNING_HOME -> FINISHED` and `RETURN_FAILED`. The saved home pose is the **search starting pose**, not the target coordinate or an origin hardcoded to (0,0). The default `navigation_enabled:=false` remains safe: no drive or return is sent unless explicitly enabled. `return_home_enabled:=false` disables automatic return.

**Robot acceptance is still required:** verify stable TF, Nav2 lifecycle and goal cancellation, real physical stop, correct saved start pose, map consistency and actual return accuracy. Nav2 Action results alone do not prove physical standstill. Do not enable live navigation without an on-site operator and safe test area.

**Before running any ROS 2 commands on the Raspberry Pi**, initialize the environment:
`source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash`


## Physical validation safety controls

The runtime defaults to non-actuating operation. When physical navigation is
explicitly enabled, the validation defaults also limit each SEARCHING mission
to one navigation goal attempt:

```yaml
navigation_enabled: false
single_goal_mode: true
max_navigation_goals: 1
```

Launch arguments expose the same controls:

```bash
ros2 launch ee3033_search_policy full_runtime.launch.py \
  navigation_enabled:=true \
  single_goal_mode:=true \
  max_navigation_goals:=1
```

In single-goal mode, a successful navigation and completed observation reports
`SINGLE_GOAL_TEST_COMPLETE`. A failed navigation reports
`SINGLE_GOAL_TEST_FAILED`. The node will not automatically dispatch another
goal in either case.

An operator can inhibit all later dispatches and request cancellation of an
active search or return-home goal with:

```bash
ros2 service call /search/cancel_navigation std_srvs/srv/Trigger "{}"
```

A successful service response means that cancellation was requested, not that
the robot has already stopped. Monitor the transient-local `/search/status`
topic for `OPERATOR_STOPPED`. Treat `OPERATOR_STOP_UNVERIFIED` as a failed
stop verification and use the separately verified hardware emergency-stop
procedure. `/search/reset_memory` refuses to rearm while the mission is `SEARCHING` or
`PAUSED`. First reset or finish the mission, verify that no navigation is
active, and then reset memory. Pause/resume preserves the current navigation
goal count.


## One-command mapped real-robot workflow

The verified interactive entry now owns the complete physical sequence:
manual SLAM, map saving, known-map Nav2 startup, AMCL initialization, camera,
NCNN YOLO, MLP search, stamped target localization, and return to the captured
mapping start pose.

Before loading the saved map, the workflow conservatively removes a singleton
occupied pixel only when all other eight cells in its complete 3x3 neighborhood
are known free. The original raster is retained as `<map-prefix>.raw.pgm`.

```bash
/home/ros2_ws/src/ee3033-search-policy/scripts/run_real_robot_workflow.sh \
  --map-prefix /home/ros2_ws/src/ee3033-search-policy/maps/map_run_YYYYMMDD_HHMM \
  --navigation-enabled true
```

The map prefix must be new. See
[`docs/REAL_ROBOT_ONE_COMMAND_WORKFLOW.md`](docs/REAL_ROBOT_ONE_COMMAND_WORKFLOW.md)
for the three operator prompts, exact reproduced parameters, recovery behavior,
logs, and build command. The workflow currently enables image-stamped
LiDAR/camera dual-mode target ranging and records the selected localization
method in the final estimate.
