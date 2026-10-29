# Phase 0: offline baseline audit and pending robot evidence

## Scope and acceptance

- Recorded: 2026-10-19 17:38 +08:00.
- Classification: offline repository audit; earlier operator-provided robot
  observations are explicitly historical.
- Operator: repository owner (robot output supplied in the integration session).
- Current stage: non-actuating route/waypoint preview from the real robot pose
  and saved map. No target, manual destination, navigation execution, or actual
  observation-memory update.
- Operator reports that the robot is disconnected and authorizes offline
  continuation with the information gaps recorded.
- Acceptance: local baseline identified; live robot audit **pending**, not passed.

## Verified local baseline

| Item | Evidence |
| --- | --- |
| Checkout | `/home/acyg/Map_Search/ee3033-search-policy` on FAGIO |
| Branch | `nav2-interface` |
| HEAD | `9abc1ada1eefea5897e553a8be818110874f13de` |
| Working tree before this record | Untracked operator-provided `AGENTS.md`; no tracked modifications |
| Cached baseline ref | `origin/runtime-policy-core` |
| Cached baseline SHA | `945f7debcfc36054d8b8320ab55ab4db8a1bb551` |
| Cached baseline subject | `Make runtime clearance estimate conservative` |
| Ancestry | Current HEAD is an ancestor of the cached runtime-core baseline |
| Additional modules in cached baseline | `runtime_viewpoints.py`, `coverage_memory.py`, `feature_builder.py`, `policy_decision.py` |
| Additional tests in cached baseline | `test/test_runtime_core.py` |
| Local test environment | Session-local Python 3.12.3, NumPy 2.5.4, pytest 9.1.1 |
| ROS/colcon in development environment | Not available at initial inspection; no ROS integration run here |

The runtime-core ref was inspected locally without fetching. Its current
upstream state has not been checked. After being offered a dedicated branch,
the operator explicitly selected continued work on `nav2-interface`, retaining
control of Git commits, pushes, and merges. This overrides the suggested
dedicated-branch workflow for this work. Preserve the untracked operator
instructions and this record. No branch change is authorized or needed.

## Earlier robot observations supplied by the operator

These are not fresh results on a runtime integration branch:

- The shell reported `root@balance:/home/ros2_ws`.
- `/amcl`, `/map_server`, `/planner_server`, Nav2 navigation nodes, `/planner_test`,
  and `/rviz` appeared in `ros2 node list`.
- `/compute_path_to_pose` was listed as
  `nav2_msgs/action/ComputePathToPose`. Navigation and other planning Actions
  were also discoverable; discovery is not a successful Action execution.
- `/map` was received using a reliable, transient-local subscription, with
  `header.frame_id: map`. A verbose publisher QoS snapshot was not supplied.
- A five-second `tf2_echo map base_footprint` initially reported missing or
  disconnected frames, then returned two valid transforms.
- Last supplied transform: translation approximately
  `(-1.534, 2.238, 0.000)` m and yaw approximately `-0.087` rad (`-4.999` degrees).
  This is historical evidence, not a usable current starting pose.
- No freshness frequency, localization covariance, or long-duration TF
  stability assessment was performed.

The earlier 57-test pass and model inference check belong to the previous
development work. They do not establish the runtime-core baseline test result.

## Information gaps and deployment gates

| Missing evidence | Why it matters | Required follow-up |
| --- | --- | --- |
| Current Pi host and Docker branch/full SHA and worktree state | Bind mount and deployed version are not freshly verified | Read-only Git/path inspection in both terminals |
| Actual ROS distribution, domain, RMW, localhost configuration | Host/container discovery must not be assumed | Capture allowlisted environment variables in each ROS terminal |
| Current map publisher QoS and map metadata | Verify live map subscription compatibility and geometry | `ros2 topic info /map --verbose`; inspect header/info without dumping cell data |
| Current map-to-base TF and localization health | Actual start must be measured and recent | Repeated TF samples, source timestamps, observed frame names, and AMCL evidence |
| Agreed and tested TF freshness thresholds | Startup buffer fill must not be confused with a valid current pose | Define offline checks, then validate against live update rates and clock settings |
| Current planner endpoint, server count, lifecycle and planning result | Discovery alone is insufficient | Inspect `/compute_path_to_pose`, planner lifecycle, then run an approved planning-only request |
| Actual robot footprint and Nav2 safety configuration | Provisional clearance is not a verified footprint | Inspect configuration before any physical motion |
| Runtime-core baseline tests | Additional modules have not been tested on this checkout | Run all tests on the explicitly approved baseline; distinguish local Python from Humble |
| Remote RViz discovery and retained displays | Local publication does not prove host visualization works | Verify host/container DDS settings and late-starting RViz after reconnect |
| Reproducible map fixture/hash | Real-map geometry/visibility regressions require recorded input | Obtain an operator-approved fixture when connected |
| Runtime metrics | No current measurements exist | Record candidate counts, timing, path outcomes, route length, CPU/memory as measured |

Camera/YOLO interfaces, target coordinates, physical navigation, and
emergency-stop validation are N/A for this non-actuating preview. They remain
separately gated before their respective future phases.

## Offline continuation constraints

1. Continue on `nav2-interface` as explicitly directed by the operator. Do not
   silently switch, fetch, merge, or commit. Reusing runtime-core source files
   on this branch is a separate implementation step, not an implied Git merge.
2. Reuse the existing runtime core and training-consistent utility/features
   rather than creating a duplicate implementation.
3. Run and report baseline tests before changing runtime behavior. A local
   test pass is not a ROS Humble/Docker build or a physical integration pass.
4. Keep preview planning explicitly non-actuating and motion disabled.
5. Keep hypothetical multi-waypoint preview state separate from actual
   `CoverageMemory`; do not mark a location observed because it was planned.
6. Freeze each preview's actual TF start. Label later segment starts as
   hypothetical; never report them as measured robot positions.
7. Do not install dependencies, alter DDS/networking, start/stop the robot stack,
   commit, or change branches without the applicable operator approval.
8. Record phase evidence and obtain acceptance before advancing. Missing robot
   evidence remains pending even if an offline portion passes.

Phase acceptance: offline audit recorded; robot portion pending.
Phase commit SHA: N/A (not committed; no automatic commit authorized).
