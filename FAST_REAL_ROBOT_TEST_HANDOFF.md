# EE3033 Fast Real-Robot Test and One-Command Launch Handoff

**Branch:** `robot-validation-hardening`  
**Robot:** `balance15`, ROS 2 Humble Docker `ros2_humble`  
**Workspace:** `/home/ros2_ws`  
**ROS_DOMAIN_ID:** `15`

This file is the primary handoff for the next physical test. Use one operator and one set of terminals. Do not ask a second teammate to start duplicate nodes.

## 1. Working rule for the assisting AI

Work in short batches:

1. stop only stale or duplicate processes;
2. start the complete stack exactly once;
3. configure the现场 target class;
4. run one real navigation goal;
5. fix the first concrete failure;
6. package the successful workflow as one command.

Do not repeat offline tests unless code changes. When the operator explicitly requests the real task and a person is beside the robot, use `navigation_enabled:=true`. Keep `single_goal_mode=true` and `max_navigation_goals=1` for the first complete run.

## 2. Physically verified interfaces

Verified on the robot on 2026-11-06:

- map: `maps/map_test_1030.yaml`;
- topics: `/map`, `/scan`, `/odom`, `/camera/image_raw`, `/camera_info`, `/robot_euler`;
- camera: 640x480, `rgb8`, frame `default_cam`;
- detector node: `/yolo_ncnn`;
- detector topic: `/yolo/detections [std_msgs/msg/String]`;
- detector rate: about 13 Hz;
- detector script: `/home/ros2_ws/scripts/yolo_ncnn_node.py`;
- model: `/home/ros2_ws/models/yolov8n_320_ncnn_model`;
- bottle was detected as COCO class 39 with confidence about 0.64-0.67;
- JSON bbox format is `[xmin, ymin, xmax, ymax]`;
- `/robot_euler` is radians: x=roll, y=pitch, z=yaw;
- odometry contains planar yaw only;
- the new dynamic TF uses roll/pitch only and does not apply Euler yaw twice;
- `map -> default_cam` was observed with finite values and live pitch changes;
- adapter input is `ncnn_json`;
- target classes come from `config/yolo_ncnn_coco80_labels.json`;
- target selection is typed service `/target/set_class`, not hard-coded bottle.

Example deployed JSON:

```json
{
  "timestamp": {"sec": 0, "nanosec": 0},
  "detections": [{
    "class_id": 39,
    "class_name": "bottle",
    "confidence": 0.66,
    "bbox": [251.0, 260.0, 320.0, 474.0]
  }]
}
```

## 3. Required repository history

Pull the branch on the robot host:

```bash
cd ~/ros2_ws/src/ee3033-search-policy
git pull --ff-only origin robot-validation-hardening
git log -8 --oneline
```

History must include:

- `593ba71` motion-interrupt localization;
- `e3669fc` operator-directed real task workflow;
- `4fd2e00` Euler roll/pitch TF;
- `3679b02` camera TF launch integration;
- `5b74fb1` NCNN input configuration;
- `831fba9` COCO label map;
- `e835404` installed configurable labels.

Preserve local map files. Never use `git clean` or `git reset --hard`.

Build inside Docker:

```bash
cd /home/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select   ee3033_search_policy_interfaces   ee3033_search_policy   --symlink-install
source /home/ros2_ws/install/setup.bash
```

## 4. Current incident to resolve first

The last run stopped before target configuration because both publishers disappeared:

```text
/camera/image_raw Publisher count: 0
/yolo/detections Publisher count: 0
/target/adapter_status = SOURCE_STALE
```

A camera restart opened `/dev/video0`, then aborted with exit code -6. Multiple people had been starting and stopping nodes. Check duplicate ownership first:

```bash
fuser -v /dev/video0 2>&1 || true
pgrep -af 'usb_cam|v4l2|camera|ffmpeg|gstreamer'
ros2 node list | grep -Ei 'usb_cam|camera'
```

Start exactly one camera. If a stale `usb_cam_node_exe` owns the device, stop only that process. If no process owns it but the driver still aborts, inspect/reconnect the USB camera or restart the Docker container once.

Do not substitute `balance_object_detector`; it is the old reference-image matcher, not the validated class detector.

## 5. Clean manual startup

### Known-map navigation

```bash
ros2 launch   balance_bringup   balance_known_map_waypoint_bringup.launch.py   map:=/home/ros2_ws/src/ee3033-search-policy/maps/map_test_1030.yaml   start_waypoints:=false
```

### Camera

```bash
ros2 launch balance_usb_cam usb_cam.launch.py mode:=detector
```

### NCNN YOLO

```bash
python3 /home/ros2_ws/scripts/yolo_ncnn_node.py
```

### Physical-robot parameters

```bash
cd /home/ros2_ws/src/ee3033-search-policy
cp config/yolo_complete_validation.yaml /tmp/ee3033_real_robot.yaml
sed -i   -e 's/exclusive_navigation_verified: false/exclusive_navigation_verified: true/'   -e 's/robot_geometry_verified: false/robot_geometry_verified: true/'   -e 's/motion_interrupt_verified: false/motion_interrupt_verified: true/'   -e 's/dynamic_camera_tf_verified: false/dynamic_camera_tf_verified: true/'   -e 's/geometry_verified: false/geometry_verified: true/g'   -e 's/assume_ground_contact: false/assume_ground_contact: true/'   /tmp/ee3033_real_robot.yaml
```

`assume_ground_contact=true` is valid only when the object rests on the floor.

### Search runtime

The launch intentionally keeps the high-rate adapter, localization, odometry,
TF and search callbacks in isolated processes. They are still started with one
launch command. Do not consolidate them into one rclpy executor: physical-robot
profiling showed sustained executor entity-scanning load and reduced NCNN rate.

```bash
cd /home/ros2_ws/src/ee3033-search-policy
ros2 launch   ee3033_search_policy   yolo_complete_validation.launch.py   params_file:=/tmp/ee3033_real_robot.yaml   navigation_enabled:=true   single_goal_mode:=true   max_navigation_goals:=1   labels_file:="$PWD/config/yolo_ncnn_coco80_labels.json"   calibration_file:="$PWD/results/updated_calibration/calibration.npz"
```

Starting the runtime does not move the robot until `/mission/start_search` is called.
After one successful single-goal result, continuous search must be explicitly
selected with `single_goal_mode:=false max_navigation_goals:=100`; do not rely
on the parameter file to override launch safety arguments.

## 6. Fast readiness check

Do not run a large audit:

```bash
for topic in   /map /scan /odom /camera/image_raw /camera_info   /yolo/detections /robot_euler
do
  printf '%-24s ' "$topic"
  ros2 topic type "$topic" 2>/dev/null || echo MISSING
done

ros2 topic echo /target/adapter_status --once   --qos-durability transient_local --qos-reliability reliable

ros2 topic echo /search/runtime_state --once   --qos-durability volatile --qos-reliability best_effort
```

Proceed only when the adapter is `WAIT_FOR_TARGET` or `TARGET_CONFIGURED`, and runtime reports `ready=true`, `quiescent=true`, and `navigation_enabled=true`.

## 7. Configure any现场 object

The target is not fixed to bottle. It must be a deployed COCO class such as `cup`, `chair`, `book`, `backpack`, `cell phone` or `teddy bear`.

For a real search, place it outside the initial camera view.

```bash
ros2 service call /mission/start_mapping std_srvs/srv/Trigger "{}"
ros2 service call /mission/finish_mapping std_srvs/srv/Trigger "{}"

ros2 service call   /target/set_class   ee3033_search_policy_interfaces/srv/SetTargetClass   "{class_name: '<TARGET_CLASS>', min_confidence: 0.55}"

ros2 service call /mission/start_search std_srvs/srv/Trigger "{}"
```

Expected sequence:

```text
IDLE -> MAPPING -> WAIT_FOR_TARGET -> SEARCHING
-> TARGET_DETECTED -> LOCALIZING_TARGET
-> TARGET_REPORTED -> FINISHED
```

Software cancel:

```bash
ros2 service call /search/cancel_navigation std_srvs/srv/Trigger "{}"
```

Final estimate:

```bash
ros2 topic echo /target/estimate --once   --qos-durability transient_local --qos-reliability reliable
```

## 8. Required one-command implementation

The current one-command workflow records `map -> base_footprint` as soon as
SLAM is ready. After loop closure the operator must drive back near that pose;
`DONE` is accepted only within 0.40 m and 0.52 rad. The saved-map phase uses
the recorded mapping-start pose for `/initialpose` and requires three
consecutive matching `/amcl_pose` and TF updates before camera/search startup.

### Current dual-mode ranging test

The current one-command workflow enables
`range_mode: nearest_lidar_then_ground` in
`config/real_robot_workflow.yaml`. Each image-stamped target observation is
matched to the closest fresh `/scan` within `0.12 s`. The bbox left/right rays
define a LiDAR angle sector with a 0.05236 rad margin. The nearest support of
at least two adjacent beams is selected without requiring a minimum
target-to-wall depth separation. LiDAR is used only when its map point agrees
with the calibrated camera ground point within `0.20 m`; otherwise the camera
point is retained. Five valid range observations are required before a
candidate is published.

Use the existing one-command entry unchanged. The final estimate's `method`
distinguishes `lidar_bbox_nearest_visual_fusion`,
`ground_projection_fusion_lidar_rejected`, and camera-only fallback. This is a
test configuration; validate the synchronization tolerance and gate against
recorded real-robot data before treating them as final calibration.

After first-frame confirmation, the one-command workflow cancels the active
NavigateToPose goal and waits for its matching remote terminal state. It then
uses the active Nav2 `/spin` server for one target-alignment turn based on the
calibrated bbox bearing. There is no explicit software angle cap; the accepted
bbox and camera field of view bound the input. A 0.17453 rad central deadband
and residual offset avoid demanding exact centering. Localization history is
reset after the Spin terminal result. Top/bottom bbox boundary contact is
LiDAR-only and is reported as `lidar_vertical_bbox_boundary_fusion` when
successful.

After one manual run succeeds, implement `real_search.launch.py` with arguments:

```text
map
target_class
min_confidence
ground_contact
navigation_enabled
start_base
start_camera
start_yolo
```

It must:

1. optionally include known-map Nav2 with `start_waypoints=false`;
2. start exactly one camera;
3. start `yolo_ncnn_node.py`;
4. launch dynamic camera TF and search runtime;
5. wait for required topics/services;
6. transition to `WAIT_FOR_TARGET`;
7. configure the typed target class;
8. start only when adapter/runtime are ready;
9. reject an unknown class before motion;
10. print target, map, label and navigation mode.

Final interface:

```bash
ros2 launch ee3033_search_policy real_search.launch.py   map:=/home/ros2_ws/src/ee3033-search-policy/maps/map_test_1030.yaml   target_class:=cup   min_confidence:=0.55   ground_contact:=true   navigation_enabled:=true
```

Do not hard-code bottle.

## 9. Minimal acceptance record

Record only:

- commit SHA and map;
- target class/threshold;
- MLP-selected goal;
- Nav2 acceptance/result;
- YOLO confidence/bbox;
- mission transitions;
- navigation cancellation terminal state;
- final estimate/method;
- measured error if available.

A successful run requires real motion, target confirmation, verified Nav2 terminal state and a final estimate. A service response alone is not completion.
