# Phase 1: robot baseline verification, 2026-11-06

## Scope and environment

- Classification: robot-side ROS package builds, existing automated tests,
  and non-starting interface inspection. This is not a ROS node integration,
  rosbag replay, or physical navigation test.
- Date: 2026-11-06; the robot container used UTC. The operator's session used
  UTC+08:00.
- Branch and robot source revision: `robot-validation-hardening` at
  `593ba719958625eeac5884d98e2c7c76b000170e`.
- Robot: Raspberry Pi 5, ROS 2 Humble Docker, `ROS_DOMAIN_ID=15`.
- Operator: present at the robot; name not recorded.
- The operator authorized the builds and tests. The known-map base and Nav2
  stack, camera and NCNN YOLO detector were already running; no search node
  or navigation goal was started for this phase.
- The Phase 0 audit record was committed separately on the development
  machine as `df0105b34be63f7ad6745e3454110919e5fda613`. This documentation
  commit was not present on the Pi during the baseline tests.

## Build and test results

All commands below were executed by the operator in the Pi ROS Docker.

```bash
source /opt/ros/humble/setup.bash
cd /home/ros2_ws
colcon build \
  --base-paths /home/ros2_ws/src/ee3033-search-policy/interface_packages/ee3033_search_policy_interfaces \
  --packages-select ee3033_search_policy_interfaces \
  --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

source /home/ros2_ws/install/setup.bash
colcon build --packages-select ee3033_search_policy --symlink-install

source /home/ros2_ws/install/setup.bash
cd /home/ros2_ws/src/ee3033-search-policy
/usr/bin/python3 -m pytest test -q
```

Observed:

- Interface package build: 1 package finished, summary time 6.46 s.
- Main package build: 1 package finished, summary time 2.90 s.
- Full existing pytest suite: **236 passed in 1.39 s**.
- No dependencies were installed during this phase.

The operator then ran:

```bash
cd /home/ros2_ws
colcon test --packages-select ee3033_search_policy \
  --return-code-on-test-failure
colcon test-result --verbose
```

The current `colcon test` output said **`Ran 0 tests`**. The test-result
summary displayed 57 tests, but `colcon test-result --all --verbose` located
them in `/home/ros2_ws/build/ee3033_search_policy/pytest.xml`; the file
modification time and XML timestamp were 2026-10-19 UTC. Those 57 results
are historical and are **not** counted as new tests from this phase. The
cause of the current colcon test discovery running zero tests was not
investigated.

## Non-starting interface inspection

```bash
source /home/ros2_ws/install/setup.bash
ros2 launch ee3033_search_policy yolo_complete_validation.launch.py --show-args
ros2 interface show ee3033_search_policy_interfaces/msg/TargetObservation
```

The launch description displayed `navigation_enabled` defaulting to `false`
and a mandatory `labels_file` argument. The installed `TargetObservation`
definition included `bearing_rad`. Displaying the launch arguments did not
start the four-node validation entry or exercise ROS callbacks, QoS, TF,
Action cancellation or target localization.

## Acceptance and limitations

- Active-revision package builds and existing pytest baseline: **PASS**.
- New `colcon test` execution: **0 tests**; the reported historical 57-test
  XML is not a current pass.
- ROS non-actuating integration, physical navigation, cancellation and
  map-frame target-coordinate accuracy: **not tested in this phase**.
- The Phase 0 blockers remain open: missing image-frame TF, unverified
  time-aligned camera pitch and source freshness, intermittent DDS/message
  errors, detector age under load, and the identified motion-interrupt and
  localization-fusion risks. See the [Phase 0 audit](phase_0_robot_audit_2026-11-06.md).
- No inference latency, candidate count, planner latency, goal result, target
  coordinate, ground truth or localization error was measured here.

Approval of this baseline record does not authorize physical motion or
continuous search. Further phases require their own operator approval and
evidence.
