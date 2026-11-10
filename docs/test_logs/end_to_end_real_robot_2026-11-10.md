# EE3033 real-robot end-to-end search and localization test

Date: 2026-11-10  
Robot: `balance15`  
ROS: ROS 2 Humble, Docker container `ros2_humble`  
Branch: `robot-validation-hardening`  
Final tested repository commit: `cff46d6`

## 1. Test outcome

The complete physical workflow succeeded:

```text
saved map + AMCL
  -> live candidate generation
  -> Tiny MLP selection
  -> repeated Nav2 navigation
  -> online YOLO bottle detection
  -> target confirmation
  -> localization and navigation inhibition
  -> matching terminal-state verification
  -> final map-frame target coordinate
  -> FINISHED
```

The final authoritative estimate was:

```yaml
frame_id: map
episode_id: 5b3dd7c3-74be-476a-bd73-af1b302d6e0f
class_id: 39
class_name: bottle
position:
  x: 0.4959790136188274
  y: -0.40947424596867377
  z: 0.0
covariance_xy:
  - 0.010397677782619168
  - -0.0004150391935578756
  - -0.00041503919355787553
  - 0.01250481444370286
quality: 0.72314453125
method: ground_projection_fusion
valid: true
reason: REPORTED_AFTER_TERMINAL_WITH_MOTION_INTERRUPT_MONITOR
```

The simplified map coordinate is therefore:

```text
bottle = (0.496, -0.409) m in frame map
```

RViz inspection showed that this result did not deviate substantially from the
physical bottle location. A measured ground-truth coordinate was not recorded,
so no numeric field error is claimed.

The `quality` field currently follows detector confidence and is not a
probability of spatial accuracy. The covariance is the localizer's estimated
uncertainty and has not yet been calibrated against enough physical
ground-truth samples.

## 2. Important experimental conclusion: one-frame confirmation

For this controlled experiment, one fresh YOLO frame was sufficient to trigger
target confirmation reliably:

```yaml
yolo_detection_adapter:
  ros__parameters:
    required_frames: 1
```

Conditions for this conclusion:

- deployed NCNN COCO model;
- bottle is COCO class 39;
- confidence threshold `0.55`;
- the bottle is visually clear and stationary on the floor;
- the camera, model, lighting and test area used on 2026-11-10;
- timestamp age and bbox validation remain enabled.

No instability or false trigger was observed in this experiment. Therefore
`required_frames: 1` is an acceptable experimental setting for this setup.
This is not yet a general claim for other classes, lighting, models or crowded
scenes.

Only the detection-to-stop trigger was reduced to one frame. Final coordinate
publication still required fresh localization candidates, consistency checks,
motion-interrupt health and the matching remote Nav2 terminal contract.

The checked-in safety default remains three frames. The successful robot
overlay used one frame.

## 3. Final runtime parameters

The successful run used these robot-specific values:

| Parameter | Value | Purpose |
| --- | ---: | --- |
| `navigation_enabled` | `true` | Enable real Nav2 goals |
| `single_goal_mode` | `false` | Continue selecting viewpoints |
| `max_navigation_goals` | `100` | Explicit continuous-run launch value |
| `return_home_enabled` | `false` | No automatic return |
| `required_frames` | `1` | Experimental one-frame confirmation |
| `min_confidence` | `0.55` | Bottle threshold |
| `max_age_s` | `0.75` | Detector source health at about 5 Hz |
| `max_gap_s` | `0.4` | Confirmation gap limit |
| `observation_position_tolerance_m` | `0.15` | Match deployed Nav2 position tolerance |
| `observation_yaw_tolerance_rad` | `0.30` | Nav2 tolerance plus balancing margin |
| `motion_interrupt_verified` | `true` | Physical overlay only |
| `dynamic_camera_tf_verified` | `true` | Physical overlay only |
| `geometry_verified` | `true` | Physical overlay only |
| `assume_ground_contact` | `true` | Valid because bottle rested on floor |
| `mission_timeout_s` | `300.0` | Bounded continuous search |

The deployed Nav2 goal checker reported:

```text
general_goal_checker.xy_goal_tolerance  = 0.15
general_goal_checker.yaw_goal_tolerance = 0.25
general_goal_checker.stateful           = true
```

The observation yaw limit was set to `0.30 rad` because the balancing robot
moved by about `0.017 rad` beyond the Nav2 limit between Nav2 success and the
observation verification callback. The measured failing sample was:

```text
distance_m=0.055
position_limit_m=0.150
yaw_error_rad=0.267
yaw_limit_rad=0.250
```

After changing the observation limit to `0.30 rad`, the single-goal acceptance
completed successfully.

## 4. Terminal ownership

Use one operator and preserve terminal ownership:

| Terminal | Owner |
| --- | --- |
| 0 | Status monitor; host-side Git update and container build when stopped |
| 1 | Robot base, saved map, AMCL and Nav2 |
| 2 | Exactly one timestamp-correct camera |
| 3 | Exactly one NCNN YOLO process |
| 4 | Exactly one EE3033 search runtime launch |
| 5 | Readiness, mission services and emergency software cancel |

Do not start duplicate camera, YOLO, Nav2 or EE3033 processes. Closing a
terminal window is not proof that its children exited. Stop a launch from its
parent terminal with `Ctrl+C`, then verify node and endpoint counts.

## 5. Updating and building

Git update must run on the Raspberry Pi host, not inside Docker:

```bash
cd /home/balance/ros2_ws/src/ee3033-search-policy
git pull --ff-only origin robot-validation-hardening
git log -1 --oneline
```

The expected tested revision is:

```text
cff46d6 Expose continuous search launch mode
```

Build inside Docker:

```bash
docker exec -it ros2_humble bash

cd /home/ros2_ws
source /opt/ros/humble/setup.bash

colcon build --packages-select \
  ee3033_search_policy_interfaces \
  ee3033_search_policy \
  --symlink-install

source /home/ros2_ws/install/setup.bash
```

Do not copy host SSH keys into the container. The host repository is mounted
into the container.

## 6. Camera requirements

The camera must:

- publish `/camera/image_raw` at 640x480 and about 30 Hz;
- publish `/camera_info`;
- use frame `default_cam`;
- use the timestamp-correct `usb_cam` overlay;
- use the verified calibration:

```text
/home/ros2_ws/src/ee3033-search-policy/results/updated_calibration/camera_info.yaml
```

Verified raw intrinsics:

```text
fx = 604.5025053922062
fy = 603.4925457708027
cx = 288.4586293037531
cy = 250.20495326829464
```

The timestamp-correct driver overlay used during development was:

```text
/home/ros2_ws/usb_cam_timestamp_test.hZhIXi/install/usb_cam
```

Source the overlay before the normal camera launch if it is not already the
active package:

```bash
source /opt/ros/humble/setup.bash
source /home/ros2_ws/usb_cam_timestamp_test.hZhIXi/install/setup.bash
source /home/ros2_ws/install/setup.bash

ros2 launch balance_usb_cam usb_cam.launch.py mode:=detector
```

Verify the active CameraInfo:

```bash
timeout 5 ros2 topic echo /camera_info --once \
  --qos-reliability best_effort
```

Do not reduce the camera to 15 Hz. Measurements showed that doing so increased
V4L2 buffering:

| Setting | Image median | Image p95 |
| --- | ---: | ---: |
| Camera 15 Hz | 261 ms | 299 ms |
| Camera 30 Hz | 53 ms | 116 ms |

The camera must remain at 30 Hz; YOLO drops intermediate images itself.

## 7. YOLO process and realtime QoS

External detector script:

```text
/home/balance/ros2_ws/scripts/yolo_ncnn_node.py
```

Container-visible path:

```text
/home/ros2_ws/scripts/yolo_ncnn_node.py
```

This script is outside the Git repository. Preserve it separately until the
detector is packaged into the repository.

There must be exactly:

```text
/camera/image_raw: 1 publisher, 1 YOLO subscriber
/yolo/detections:  1 YOLO publisher, 1 adapter subscriber
```

The script must contain only one image subscription and one detection
publisher. A previous edit accidentally retained both reliable and
best-effort endpoints, causing every image to be inferred twice.

The realtime QoS used by YOLO was:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

realtime_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

self.publisher = self.create_publisher(
    String,
    "/yolo/detections",
    realtime_qos,
)

self.subscription = self.create_subscription(
    Image,
    "/camera/image_raw",
    self.image_callback,
    realtime_qos,
)
```

YOLO was intentionally limited to about 5-6 Hz while the camera remained at
30 Hz:

```python
self.inference_period = 1.0 / 6.0
self.last_inference_start = 0.0
```

At the start of the callback:

```python
def image_callback(self, msg):
    now = time.monotonic()
    if now - self.last_inference_start < self.inference_period:
        return

    self.last_inference_start = now

    try:
        # Convert and infer only after the rate gate.
        ...
```

Do not use `time.sleep()` inside the callback. Returning before conversion and
inference allows DDS depth 1 to retain the latest sample without blocking the
executor.

The deployed JSON includes the original image timestamp:

```python
output.data = json.dumps({
    "timestamp": {
        "sec": msg.header.stamp.sec,
        "nanosec": msg.header.stamp.nanosec,
    },
    "detections": detections,
})
```

Start exactly one detector:

```bash
python3 /home/ros2_ws/scripts/yolo_ncnn_node.py
```

Measured after rate limiting:

```text
JSON n=59
min=64.4 ms
median=125.0 ms
p95=357.7 ms
max=522.7 ms
over 500 ms=1
```

At about 5 Hz, a new result arrives every 200 ms. The adapter health window was
therefore increased from `0.50 s` to `0.75 s` in the robot overlay. A stale
result is still rejected; the wider window prevents gaps between otherwise
fresh 5 Hz results from momentarily failing the mission.

## 8. Realtime QoS policy

High-rate, freshness-based streams use:

```text
BEST_EFFORT
VOLATILE
KEEP_LAST
depth 1
```

This includes:

- `/camera/image_raw`;
- `/yolo/detections`;
- `/target/observation`;
- `/target/estimate_candidate`;
- `/odom`;
- `/robot_euler`;
- `/mission/lease`;
- `/search/runtime_state`.

Configuration, mission state, reset, final target result and Nav2 actions remain
reliable. Do not convert every topic to best effort.

## 9. Known-map Nav2 startup

Terminal 1:

```bash
ros2 launch balance_bringup \
  balance_known_map_waypoint_bringup.launch.py \
  map:=/home/ros2_ws/src/ee3033-search-policy/maps/map_test_1030.yaml \
  start_waypoints:=false
```

Confirm the actual map file:

```bash
ros2 param get /map_server yaml_filename
```

Expected:

```text
/home/ros2_ws/src/ee3033-search-policy/maps/map_test_1030.yaml
```

Verified hashes:

```text
map_test_1030.yaml
3610cd1b645293afbbf8553e6eeff78cb7d4ab27869c2082142fd67df0fa0a42

map_test_1030.pgm
85481534919a9cbc815d6f2138258564023690b246352147359ef8467b4a5643
```

In RViz, use `2D Pose Estimate` to place the robot at its real map position and
heading. Confirm that the laser scan aligns with the map.

Check:

```bash
timeout 4 ros2 run tf2_ros tf2_echo map base_footprint
ros2 action info /navigate_to_pose
ros2 topic info /map
```

Required:

- finite and plausible `map -> base_footprint`;
- one `/navigate_to_pose` server;
- one `/map` publisher.

## 10. Robot parameter overlay

Create the physical overlay from the checked-in safe defaults:

```bash
cd /home/ros2_ws/src/ee3033-search-policy
cp config/yolo_complete_validation.yaml /tmp/ee3033_real_robot.yaml

sed -i \
  -e 's/exclusive_navigation_verified: false/exclusive_navigation_verified: true/' \
  -e 's/robot_geometry_verified: false/robot_geometry_verified: true/' \
  -e 's/motion_interrupt_verified: false/motion_interrupt_verified: true/' \
  -e 's/dynamic_camera_tf_verified: false/dynamic_camera_tf_verified: true/' \
  -e 's/geometry_verified: false/geometry_verified: true/g' \
  -e 's/assume_ground_contact: false/assume_ground_contact: true/' \
  -e 's/^    required_frames: 3$/    required_frames: 1/' \
  -e 's/^    max_age_s: 0.5$/    max_age_s: 0.75/' \
  -e 's/^    observation_yaw_tolerance_rad: 0.25$/    observation_yaw_tolerance_rad: 0.30/' \
  /tmp/ee3033_real_robot.yaml
```

Inspect the effective overlay:

```bash
grep -n -E \
  'verified|required_frames|max_age_s|observation_.*tolerance|assume_ground_contact' \
  /tmp/ee3033_real_robot.yaml
```

Do not set `assume_ground_contact=true` for a hand-held or elevated target.

## 11. Continuous search runtime

Terminal 4:

```bash
cd /home/ros2_ws/src/ee3033-search-policy
source /opt/ros/humble/setup.bash
source /home/ros2_ws/install/setup.bash

ros2 launch ee3033_search_policy \
  yolo_complete_validation.launch.py \
  params_file:=/tmp/ee3033_real_robot.yaml \
  navigation_enabled:=true \
  single_goal_mode:=false \
  max_navigation_goals:=100 \
  labels_file:="$PWD/config/yolo_ncnn_coco80_labels.json" \
  calibration_file:="$PWD/results/updated_calibration/calibration.npz"
```

Continuous mode must be passed explicitly. Before commit `cff46d6`, the launch
hard-coded single-goal mode and overrode the YAML file.

Confirm effective parameters:

```bash
ros2 param get /search_manager single_goal_mode
ros2 param get /search_manager max_navigation_goals
ros2 param get /search_manager observation_position_tolerance_m
ros2 param get /search_manager observation_yaw_tolerance_rad
ros2 param get /yolo_detection_adapter required_frames
ros2 param get /yolo_detection_adapter max_age_s
```

Expected:

```text
False
100
0.15
0.30
1
0.75
```

## 12. Fast readiness check

Terminal 5:

```bash
for topic in \
  /map /scan /odom /camera/image_raw /camera_info \
  /yolo/detections /robot_euler
do
  printf '%-24s ' "$topic"
  ros2 topic type "$topic" 2>/dev/null || echo MISSING
done

ros2 action info /navigate_to_pose

timeout 5 ros2 topic echo /target/adapter_status --once \
  --qos-durability transient_local \
  --qos-reliability reliable

timeout 5 ros2 topic echo --full-length /search/runtime_state --once \
  --qos-durability volatile \
  --qos-reliability best_effort

timeout 5 ros2 topic echo --full-length /mission/diagnostics --once \
  --qos-durability transient_local \
  --qos-reliability reliable
```

Required before starting:

```text
adapter = WAIT_FOR_TARGET or TARGET_CONFIGURED
ready = true
reason = READY
quiescent = true
navigation_enabled = true
operator_stop = false
unknown_terminal = false
motion_interrupt_verified = true
odom_healthy = true
```

## 13. Mission setup and one-call continuous search

Place the bottle on the floor, within the map, outside the initial camera view
but visible from at least one reachable viewpoint. Personnel must leave the
robot motion area before starting.

Terminal 5:

```bash
timeout 5 ros2 service call /search/reset_memory \
  std_srvs/srv/Trigger "{}"

ros2 service call /mission/start_mapping \
  std_srvs/srv/Trigger "{}"

ros2 service call /mission/finish_mapping \
  std_srvs/srv/Trigger "{}"

ros2 service call /target/set_class \
  ee3033_search_policy_interfaces/srv/SetTargetClass \
  "{class_name: 'bottle', min_confidence: 0.55}"
```

The mapping services are mission phase acknowledgements. They do not start or
save SLAM.

Check:

```bash
timeout 3 ros2 topic echo /target/adapter_status --once \
  --qos-durability transient_local \
  --qos-reliability reliable

timeout 3 ros2 topic echo --full-length /search/runtime_state --once \
  --qos-durability volatile \
  --qos-reliability best_effort
```

Expected:

```text
TARGET_CONFIGURED
ready=true
quiescent=true
finished=false
goal_count=0
```

Start exactly once:

```bash
ros2 service call /mission/start_search \
  std_srvs/srv/Trigger "{}"
```

Do not write a shell loop around `start_search`. The search manager automatically
selects, plans, navigates, observes and selects the next viewpoint.

Expected sequence:

```text
SEARCHING
  -> PLANNING_CANDIDATES
  -> NAVIGATING
  -> OBSERVING
  -> READY
  -> repeat
  -> TARGET_DETECTED
  -> LOCALIZING_TARGET
  -> TARGET_REPORTED
  -> FINISHED
```

`OBSERVING` is the coverage-memory dwell after reaching a viewpoint. It is not
a gate that turns YOLO on. YOLO processes frames throughout the whole
`SEARCHING` state, including planning and navigation.

In the successful continuous run the robot executed at least 13 navigation
goals before the target was confirmed.

## 14. Lightweight transition monitor

Terminal 0, inside Docker:

```bash
source /opt/ros/humble/setup.bash
source /home/ros2_ws/install/setup.bash

python3 - <<'PY'
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String

rclpy.init()
node = Node("search_transition_probe")

realtime = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
latched = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

last = {}

def changed(name, value):
    if last.get(name) != value:
        last[name] = value
        print(time.strftime("%H:%M:%S"), name, value, flush=True)

def lease(msg):
    data = json.loads(msg.data)
    changed("LEASE", (
        data.get("state"),
        data.get("camera_ready"),
        data.get("episode"),
        data.get("stop_token"),
    ))

def adapter(msg):
    changed("ADAPTER", msg.data)

def runtime(msg):
    data = json.loads(msg.data)
    changed("RUNTIME", (
        data.get("ready"),
        data.get("reason"),
        data.get("status"),
        data.get("operator_stop"),
        data.get("finished"),
        data.get("goal_count"),
        data.get("terminal"),
    ))

node.create_subscription(String, "/mission/lease", lease, realtime)
node.create_subscription(String, "/target/adapter_status", adapter, latched)
node.create_subscription(String, "/search/runtime_state", runtime, realtime)

try:
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
```

Do not run continuous `ros2 topic hz` probes during the real task. They add
subscriptions and Python processing load.

## 15. Final result retrieval

Retrieve the authoritative result before restarting Terminal 4:

```bash
timeout 5 ros2 topic echo --full-length /target/estimate --once \
  --qos-durability transient_local \
  --qos-reliability reliable

timeout 5 ros2 topic echo --full-length /target_position --once \
  --qos-durability transient_local \
  --qos-reliability reliable

timeout 5 ros2 topic echo /target_found --once \
  --qos-durability transient_local \
  --qos-reliability reliable
```

`/target/estimate` is authoritative. Confirm:

- `frame_id=map`;
- `episode_id` matches `/target/configuration`;
- `class_id=39`;
- `class_name=bottle`;
- `valid=true`;
- `reason=REPORTED_AFTER_TERMINAL_WITH_MOTION_INTERRUPT_MONITOR`.

A transient-local sample is retained by the live publisher. Restarting the
runtime destroys the old publisher's cache. The current coordinator does not
log the numeric coordinate, so retrieve or record it before restart.

## 16. RViz display

On the RViz machine:

```bash
source /opt/ros/humble/setup.bash
source ~/ee3033_ws/install/setup.bash
rviz2
```

Set:

```text
Fixed Frame = map
```

Add:

```text
/map             -> Map
/target_position -> PointStamped
```

For `/target_position`, use reliable and transient-local QoS.

The successful point was:

```text
x = 0.4959790136188274
y = -0.40947424596867377
z = 0.0
```

If a PointStamped display is unavailable, publish a marker:

```bash
ros2 topic pub -r 2 /target_marker visualization_msgs/msg/Marker \
"{header: {frame_id: map}, ns: bottle_target, id: 0, type: 2, action: 0, pose: {position: {x: 0.4959790136188274, y: -0.40947424596867377, z: 0.10}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, scale: {x: 0.20, y: 0.20, z: 0.20}, color: {r: 1.0, g: 0.0, b: 0.0, a: 1.0}}"
```

## 17. Stop procedure

Software cancel:

```bash
ros2 service call /search/cancel_navigation \
  std_srvs/srv/Trigger "{}"
```

Service success means a cancel request was accepted locally. It is not proof
that the robot stopped. Confirm:

- matching remote Nav2 terminal state;
- `quiescent=true`;
- no unknown terminal state;
- physical robot stop.

The physical emergency stop remains the primary response to unsafe motion.

## 18. Problems found and fixes applied

### Duplicate YOLO endpoints

The external script temporarily had two image subscriptions and two detection
publishers. This ran NCNN inference twice per frame. The final script has one
best-effort depth-1 input and one best-effort depth-1 output.

### Raw image duplicate subscription

The adapter previously subscribed to the full raw image only to retain header
metadata. It now uses matching CameraInfo metadata instead.

Commit:

```text
0d318e1 Avoid duplicate raw image subscription
```

### Shared Python executor CPU load

Five project nodes in one `SingleThreadedExecutor` caused sustained entity
scanning and callback load. The launch now starts isolated processes.

Commit:

```text
0b82b17 Isolate high-rate search callbacks
```

### Idle Nav2 timeout timers

Nav2 action timeout timers previously woke the executor while no request was
active. They now remain canceled until a request starts.

Commit:

```text
0774a53 Suspend idle Nav2 timeout timers
```

### Detection age diagnosis

Rejected JSON now reports the actual source age.

Commit:

```text
19ae62e Report rejected detection source age
```

### Realtime QoS

Freshness-based streams now use best effort, volatile, depth 1.

Commit:

```text
d687697 Prefer latest samples on realtime streams
```

### New-episode rearm race

An old `finished=true` could be observed immediately after a new search entered
`SEARCHING`. New episodes now rearm in `WAIT_FOR_TARGET`, and the coordinator
refuses start until the runtime reports rearmed.

Commit:

```text
40adabf Rearm search before new episode starts
```

### Nav2 versus observation tolerance mismatch

Nav2 could report success at its configured tolerance while the project used a
stricter hard-coded observation yaw limit. Observation tolerances are now
parameters and failures print distance, yaw, camera and health details.

Commit:

```text
b855d04 Align observation checks with Nav2
```

### Continuous mode overridden by launch

The validation launch hard-coded `single_goal_mode=true` and
`max_navigation_goals=1`, overriding the YAML file. These are now explicit
launch arguments with safe defaults.

Commit:

```text
cff46d6 Expose continuous search launch mode
```

## 19. Expected nonfatal messages

Some candidate poses can produce:

```text
Action finished with status 6; Nav2 returned an empty path
```

This means the planner could not produce a path to that candidate. The manager
records the candidate failure and continues with another reachable candidate.
It is not fatal while a later candidate is selected.

An occasional adapter transition:

```text
SOURCE_STALE -> TARGET_CONFIGURED
```

is acceptable before the active search if it immediately recovers. Persistent
staleness during `SEARCHING` fails closed.

## 20. Validation performed

Offline validation after the final code changes:

```text
python3 -m compileall -q ee3033_search_policy launch test
python3 -m pytest test -q
251 passed
```

Physical validations completed:

- correct `map_test_1030` map and AMCL pose;
- one map publisher and one Nav2 action server;
- timestamp-correct 30 Hz camera;
- single YOLO publisher/subscriber chain;
- about 5 Hz rate-limited NCNN inference;
- realtime QoS compatibility;
- single-goal navigation success;
- continuous multi-goal search;
- Tiny MLP selections;
- target confirmation;
- immediate localization state transition;
- final target report after terminal-state verification;
- map-frame bottle coordinate displayed in RViz.

## 21. Remaining work

- Package the external YOLO script into a reproducible repository-managed node.
- Implement the final top-level `real_search.launch.py` that owns base, map,
  camera, YOLO, target configuration and search readiness without duplicates.
- Record a synchronized rosbag for cancellation during active navigation.
- Measure multiple physical ground-truth target positions and report mean,
  maximum and percentile localization error.
- Re-test one-frame confirmation with other classes, lighting and distractors
  before changing the repository-wide default.
- Consider logging the final numeric target coordinate so a runtime restart
  does not remove the only readily accessible copy.
