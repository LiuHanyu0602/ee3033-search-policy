# Phase 6: first single-goal physical navigation, 2026-11-06

## Scope

- Classification: physical robot motion.
- Operator location: FAGIO SSH session into the Raspberry Pi ROS Docker
  container; personnel and the physical emergency-stop method were reported
  ready.
- Robot branch: `robot-validation-hardening`.
- Robot source revision:
  `e3669fc50349a3d23d862fdaf6219689ed94e803`.
- Development-machine revision at the time of this record:
  `30fbc3e7374d0abfd9c774d3ef313d7ec5f07916`.
- The robot working tree retained the operator's deleted tracked
  `balance_map_1` files and untracked field maps. They were not changed.
- This was one direct Nav2 goal. The search manager, automatic loop and
  automatic return were not started.

## Pre-dispatch observations

The `/navigate_to_pose` Action reported one server (`/bt_navigator`) and two
clients (`/bt_navigator` and `/waypoint_follower`). The planner and controller
lifecycle nodes both reported `active [3]`. The filtered node list contained
the expected Nav2 servers and waypoint follower, with no search, mission or
exploration node observed.

The initial `map -> base_footprint` samples were approximately:

- position `(0.312, 0.262)` m;
- yaw `0.982` rad;
- quaternion `(0, 0, 0.471, 0.882)`.

The selected first goal was 0.30 m along that heading:

- position `(0.479, 0.511)` m;
- quaternion `(0, 0, 0.471, 0.882)`;
- frame `map`.

The exact goal was submitted to `/compute_path_to_pose` first. Nav2 accepted
planning goal UUID `a5c8a76d-ed93-4e42-aa61-b3bdc71d4e5c`, returned 13 poses,
reported planning time `0.000210112 s`, and finished `SUCCEEDED`.

## Physical navigation

The operator then submitted exactly one direct `NavigateToPose` goal:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: map}, pose: {position: {x: 0.479, y: 0.511, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.471, w: 0.882}}}}" \
--feedback
```

The provided feedback included:

- elapsed navigation time `5.740200745 s`;
- zero recoveries;
- reported distance remaining `0.191067159 m`;
- current pose approximately `(0.384, 0.351)` m at yaw approximately
  `0.849` rad.

The Action command returned `Goal finished with status: SUCCEEDED`. The
operator separately reported that physical movement completed and the robot
stopped.

Post-run `/navigate_to_pose/_action/status` identified navigation goal UUID
`b6b31593-9092-4b13-bb30-4da249b8ab3f` with status `4` (`SUCCEEDED`).
Post-run TF samples were approximately `(0.398, 0.345, 0.765)` and
`(0.395, 0.342, 0.788)` for `(x, y, yaw)`.

Using the latter sample, net displacement from the initial sample was about
0.115 m. Remaining position error to the requested goal was about 0.189 m and
heading error about 0.194 rad. Therefore the result demonstrates Nav2
completion within its configured goal tolerances, not exact arrival at the
requested coordinates.

## Decision and remaining scope

**Partial Phase 6 pass:** one short direct Nav2 goal reached a remote
`SUCCEEDED` terminal state, and the operator confirmed physical stop. No
second goal was sent.

This run does not validate:

- MLP candidate selection or the guarded search-manager dispatch path;
- repeated navigation or coverage-memory updates;
- target detection during motion or navigation cancellation;
- the operator-cancel software path;
- target-coordinate estimation;
- complete Stage 2 execution.

`tf2_echo` again printed `sequence size exceeds remaining buffer` during the
pre- and post-run observations. TF samples continued afterward, but the
transport/deserialization symptom remains unresolved.

Map identifier/hash, CPU and memory utilization, controller limits, exact
Nav2 goal-tolerance parameters, and ground-truth displacement were not
measured in this run.

## Follow-up: MLP selection and one navigation goal

The operator then authorized one integrated policy-selection navigation test.
This used the base `search_manager` on the same robot revision, with explicit
runtime overrides:

```bash
ros2 run ee3033_search_policy search_manager --ros-args \
  -p navigation_enabled:=true \
  -p return_home_enabled:=false \
  -p single_goal_mode:=true \
  -p max_navigation_goals:=1
```

The node reported navigation enabled, a one-goal maximum, and 96 generated
candidate poses. Before search authorization, the operator accidentally
called `/search/cancel_navigation`. It returned `success=True` with
`No active navigation goal; future dispatch is inhibited`; no goal was active
and no motion occurred. While the mission was still `IDLE`,
`/search/reset_memory` returned `success=True` and `Search memory reset`.
The operator then published `SEARCHING` once using reliable,
transient-local QoS.

The search-manager log reported:

```text
Candidate 84 failed: Action finished with status 6; Nav2 returned an empty path
Selected 40 by tiny_mlp; path=0.66m
```

Candidate 84 was a failed planning attempt, not a navigation goal. Candidate
40 was selected by the Tiny MLP and produced the only navigation goal in this
run. The operator reported that the robot navigated to the new point.

The post-run Nav2 Action status contained goal UUID
`7be9929d-1092-429c-aaa3-b40d49071d7c` with status `4` (`SUCCEEDED`).
Post-run TF samples were approximately `(-0.153, 0.522, -1.243)` and
`(-0.136, 0.473, -1.243)` for `(x, y, yaw)`. Relative to the end of the
previous short-goal run, the latter sample represents approximately 0.56 m
net displacement, consistent with the reported 0.66 m planned path.

Attempts to read the latched `/search/selected_goal` and `/search/status`
messages printed only `sequence size exceeds remaining buffer`. Consequently,
this run verifies the Tiny MLP selection log, physical motion, operator
observation and Nav2 terminal status, but not successful DDS receipt of those
two search topics. Coverage-memory update after the observation timer was not
independently observed.
