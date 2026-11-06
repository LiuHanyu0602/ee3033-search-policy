# Phase 0: robot interface audit, 2026-11-06

## Scope and decision

- Classification: non-actuating robot interface audit and raw detector
  observation. The base was powered for balancing; no navigation goal was
  sent during this audit.
- Date: 2026-11-06. The robot container reported UTC; the operator's session
  used UTC+08:00.
- Operator: present at the robot; name not recorded. The operator confirmed
  an available physical emergency-stop or power-off method.
- Branch: `robot-validation-hardening`.
- Audited robot revision: `593ba719958625eeac5884d98e2c7c76b000170e`.
  The Pi was fast-forwarded from `9e87b3f88bd97a1a6f5be21581739573fc4c45e9`
  with operator approval. Existing deleted tracked maps and untracked maps
  were preserved.
- Decision: interface audit recorded with open blockers. This is not
  acceptance of localization, navigation, target reporting, or physical
  motion. Robot-side baseline build and tests have not been run on this
  revision.

## Environment and active interfaces

- ROS 2 Humble, `ROS_DOMAIN_ID=15`, `ROS_LOCALHOST_ONLY=0`; the container
  did not report an explicit `RMW_IMPLEMENTATION` environment value.
- The operator authorized the existing known-map base and Nav2 bringup with
  `start_waypoints:=false`, followed separately by the timestamp-repair
  camera and existing NCNN YOLO detector. The operator reported the base
  remained controlled and did not move on its own.
- The observed node list included AMCL, map server, planner, controller,
  LiDAR and base. It did not include `continuous_waypoint_loop`. Nav2's
  `waypoint_follower` was present; its presence alone does not establish an
  active waypoint loop.
- `/compute_path_to_pose`: `nav2_msgs/action/ComputePathToPose`, server
  `/planner_server`. `/navigate_to_pose`: `nav2_msgs/action/NavigateToPose`,
  server `/bt_navigator`. No action goal was submitted.
- `/amcl`, `/planner_server` and `/controller_server` reported lifecycle
  state `active`. The map server reported the selected map YAML path below.
- `/map`: `nav_msgs/msg/OccupancyGrid`, reliable, transient-local publisher.
  `/scan`: `sensor_msgs/msg/LaserScan`, reliable, volatile publisher; observed
  frame `laser` and approximately 10 Hz. `/odom`: `nav_msgs/msg/Odometry`,
  reliable, volatile publisher; observed `odom_combined` to `base_footprint`
  and approximately 50 Hz. Existing best-effort sensor subscribers are
  compatible with these reliable publishers.
- `/camera/image_raw` and `/camera_info` were reliable and volatile;
  observed frame was `default_cam`. Image rate was approximately 24–29 Hz in
  the sampled interval. `/yolo/detections` was reliable, volatile
  `std_msgs/msg/String` JSON with the original input image timestamp.
- `/robot_euler` was `geometry_msgs/msg/Vector3Stamped`, published by
  `/balance_robot_node` with frame `base_footprint`. The inspected base source
  converts STM32 roll, pitch and yaw from degrees to radians. It sets the
  publication timestamp before reading the serial sensor data. The sensor
  read returns whether new data was received, but the control loop does not
  check that return value before publishing; source-data freshness therefore
  cannot be inferred solely from the ROS timestamp. A serial outage was not
  deliberately tested.

## Map and models

The map server reported
`/home/ros2_ws/src/ee3033-search-policy/maps/map_test_1030.yaml`. The YAML
specifies resolution 0.05 m and origin `[-0.881, -1.27, 0]`. The physical
accuracy of the current AMCL pose against the saved map was not verified.

| File | SHA256 |
| --- | --- |
| `maps/map_test_1030.yaml` | `3610cd1b645293afbbf8553e6eeff78cb7d4ab27869c2082142fd67df0fa0a42` |
| `maps/map_test_1030.pgm` | `85481534919a9cbc815d6f2138258564023690b246352147359ef8467b4a5643` |
| `models/tiny_search_policy_weights.npz` | `3f2c9e784f71d3d5921f8e699d0013ca377034ff70d6f87722b73062d132a3b5` |

The MLP hash matches the expected frozen-weights hash; inference and runtime
selection were not tested. The NCNN model metadata and the existing label
JSON each contained 80 classes and compared equal. NCNN model file hashes were
not recorded.

## TF and camera geometry

- `map` to `base_footprint` and `map` to `camera_link` were observed.
  `map` to the image frame `default_cam` was not obtained during the sampled
  lookup; it must not be used for target localization without verification.
- The existing static transforms reported `base_footprint` to `laser` as
  `(0.002, 0, 0.1875)` m and `base_footprint` to `camera_link` as
  `(-0.035, 0, 0.2655)` m, both with identity rotation. The observed
  `map` to `camera_link` samples had zero roll and pitch; this does not verify
  camera optical axes or compensate physical balancing pitch.
- The operator described the forward-looking camera as approximately 5 cm
  above and 2 cm behind the LiDAR, without lateral offset, estimated from
  device housings rather than sensor origins. Assuming the existing laser TF
  and forward-positive x, this suggests an approximate camera position
  `(-0.018, 0, 0.2375)` m. It is not a calibrated optical-center transform
  and has not been installed or validated.
- The operator confirmed the camera is fixed to the pitching robot body.
  Observed IMU orientation covariance had `1e6` for the first two diagonal
  entries. IMU and Euler axis conventions, sign, source freshness and
  image-time alignment remain unverified. No new TF was published.

## Detector observations and resource limits

- A raw YOLO message reported `bottle`, class ID 39, confidence approximately
  0.663, with a bounding box and image timestamp. Other individual messages
  contained a lower-confidence bottle or no detection. This is not
  multi-frame confirmation or a coordinate estimate.
- One eight-second read-only subscriber observed 105 messages, no messages
  meeting all of the provisional confidence `>=0.65`, image-age `<=0.5 s`
  and increasing-timestamp criteria, and image ages up to approximately
  1.31 s. A separate eight-second sample observed 104 messages, 94 with a
  bottle, 32 with confidence `>=0.65`, 97 with age in `0..0.5 s`, no
  non-increasing timestamps, and maximum age approximately 1.14 s. The latter
  sample did not calculate the intersection or consecutive-run length.
  These temporary observation thresholds are not an acceptance decision.
- With the detector running, one four-core snapshot showed load average
  7.64 and about 85% non-idle CPU. The operator reported that balancing
  control is handled by a separate STM32 and that the base remained
  controlled. Pi scheduling, perception, planner and stop-handshake latency
  under sustained load remain unverified.
- `sequence size exceeds remaining buffer` appeared intermittently in
  multiple ROS commands and process logs; message-loss notifications also
  appeared. Their cause and impact are unknown. Do not treat successful
  samples or topic rates as proof of healthy communication.

## Commands used

Commands were run by the operator on the Pi host or in the Pi ROS Docker,
as appropriate. No automatic navigation or velocity command was issued.

```bash
# Pi host: audit and operator-approved fast-forward
git status --short --branch
git fetch -q origin robot-validation-hardening
git diff --name-only HEAD origin/robot-validation-hardening
git merge-base --is-ancestor HEAD origin/robot-validation-hardening
git merge --ff-only 593ba719958625eeac5884d98e2c7c76b000170e

# Pi ROS Docker: interface, map, TF and sensor audit
ros2 node list
ros2 topic list -t
ros2 action list -t
ros2 action info /compute_path_to_pose
ros2 action info /navigate_to_pose
ros2 topic info /map --verbose
ros2 topic info /scan --verbose
ros2 topic info /odom --verbose
ros2 topic info /camera/image_raw --verbose
ros2 topic info /camera_info --verbose
ros2 topic info /yolo/detections --verbose
ros2 topic info /robot_euler --verbose
ros2 lifecycle get /amcl
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 param get /map_server yaml_filename
timeout 8 ros2 run tf2_ros tf2_echo map base_footprint
timeout 8 ros2 run tf2_ros tf2_echo map default_cam
timeout 8 ros2 run tf2_ros tf2_echo base_footprint camera_link
timeout 8 ros2 run tf2_ros tf2_echo base_footprint laser
timeout 8 ros2 run tf2_ros tf2_echo map camera_link
sha256sum maps/map_test_1030.yaml maps/map_test_1030.pgm \
  models/tiny_search_policy_weights.npz
```

Additional read-only samples included topic headers, rates, IMU covariance,
base-source inspection, model-label comparison, CPU status and two short
raw-detection subscriber audits. Full DDS tracing, ground truth, CPU
time-series and rosbag capture were not performed.

## Open blockers and next gate

1. Establish the image optical frame, approximate extrinsics and
   time-aligned dynamic pitch from a source with verifiable freshness; do
   not enable geometry verification or publish a formal target coordinate
   merely to make the workflow advance.
2. Diagnose the recurring serialization/message-loss reports and verify
   sensor and detector freshness under load.
3. Reproduce and address the reported motion-interrupt, delayed-frame and
   odometry-gap fusion risks. The latest motion-interrupt design also removes
   the previously required fresh post-terminal odometry window; it is not
   accepted for physical validation on that basis.
4. Run and inspect the active-revision baseline tests and ROS non-actuating
   integration checks after the operator approves the next phase.
5. Before any motion, verify physical map localization, footprint and safety
   configuration, emergency stop and a single short goal. Separately verify
   remote terminal state and cancellation before considering multiple goals.
   The new validation launch currently forces a single-goal budget.

Candidate count, path planning latency, inference latency, target coordinates,
ground truth, localization error and new-revision navigation results: not
measured. Historical navigation results are not fresh acceptance evidence.
