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

## Target Detection and Stop Integration

### Overview

The target detection and stop integration modules connect the Raspberry Pi's YOLOv8n-320 NCNN detector to the autonomous search controller.

The detection logic is implemented separately from the MLP waypoint-selection algorithm.

### ROS 2 Interfaces

| Topic | Message Type | Purpose |
|---|---|---|
| `/yolo/detections` | `std_msgs/msg/String` | JSON detection results published by the YOLO NCNN node |
| `/target_found` | `std_msgs/msg/Bool` | Confirmed target-detection status |

### Target Checker

`target_checker.py` subscribes to `/yolo/detections` and validates incoming detections.

Default configuration:

- Target class: `bottle`
- Minimum detection confidence: `0.70`
- Required consecutive detection frames: `3`
- Output frequency: `5 Hz`

The checker verifies distinct frame timestamps to avoid counting duplicate images.

Once the target is confirmed, `/target_found` remains `True` until the node is restarted.

Run the checker after building and sourcing the ROS 2 workspace:

```bash
ros2 run ee3033_search_policy target_checker
```

### Stop Integration

`target_stop_guard.py` provides stop-handling logic for the autonomous search controller.

When `/target_found=True` is received, the search controller should:

1. Stop issuing new search waypoints.
2. Request cancellation of the active Nav2 `NavigateToPose` goal.
3. Check the final navigation action result.
4. Verify that the robot has stopped moving.

The stop guard must be integrated into the same controller that owns the active Nav2 navigation request. It is not a standalone executable.

### Current Integration Status

The Target Checker implementation is available, but end-to-end detection and physical stopping have not yet been validated.

Integration with the runtime MLP search controller is still required before autonomous stop behavior can be confirmed.


## Full ROS 2 runtime interface (development branch)

The `full-runtime-integration` branch contains the complete software-facing
Stage-2 loop. It is intentionally safe by default: the search manager publishes
its selected goal but cannot send NavigateToPose unless
`navigation_enabled:=true` and `/mission/state` is `SEARCHING`.

### Runtime data flow

1. `/map` and `map -> base_footprint` generate safe oriented viewpoints.
2. Coverage memory computes new visible area for every viewpoint.
3. Nav2 `/compute_path_to_pose` supplies real path distances.
4. The training-equivalent 32-candidate shortlist and 12 features are built.
5. Tiny MLP selects a viewpoint; invalid inference falls back to the immediate
   time-aware score.
6. Nav2 executes one goal. Coverage is committed only after success and the
   observation dwell time.
7. Navigation failure counts temporarily blacklist repeated unreachable goals.
8. A confirmed target latches `/target_found`, cancels the active goal and
   prevents all later dispatch.
9. RGB bearing plus LaserScan (or calibrated ground-plane fallback) publishes
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

- `candidate_clearance_m` versus the real footprint and Nav2 inflation;
- `camera_pitch_down_rad` and camera optical-axis convention;
- `camera_yaw_offset_rad` and `laser_yaw_offset_rad`;
- whether the 2-D laser intersects the target rather than the wall behind it;
- camera FOV/range and navigation speed/time assumptions;
- the robot-specific LED or buzzer command interface.

Until calibration, `target_position_method` must be logged with every reported
coordinate and the target coordinate must be treated as provisional.
