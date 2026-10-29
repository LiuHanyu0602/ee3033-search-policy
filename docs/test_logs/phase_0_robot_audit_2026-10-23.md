# Phase 0: robot audit, 2026-10-23

## Scope

- Classification: read-only robot environment audit, operator-reported output.
- Session report time: approximately 14:06 +08:00.
- Robot SSH address supplied by operator: `192.168.1.115`.
- No motion, launch, build, package installation, or Git modification performed
  as part of this audit.

## Observed results

| Item | Operator-reported evidence |
| --- | --- |
| Pi host repository | `/home/balance/ros2_ws/src/ee3033-search-policy` |
| Docker repository | `/home/ros2_ws/src/ee3033-search-policy` |
| Both branches | `nav2-interface`, tracking `origin/nav2-interface` |
| Both revisions | `9abc1ada1eefea5897e553a8be818110874f13de` |
| Both working trees | No changes reported by `git status --short --branch` |
| ROS environment | `/opt/ros/humble/setup.bash` sourced successfully |
| `ROS_DISTRO` | `humble` |
| `ROS_DOMAIN_ID` | `15` |
| `ROS_LOCALHOST_ONLY` | `0` |
| `RMW_IMPLEMENTATION` | Not present in filtered environment output; effective implementation not measured |
| `ros2 node list` | No nodes listed |
| Initial overlay source attempt | `source install/setup.bash` from `/` failed because the relative file was absent |

The failed relative source command does not establish whether the workspace
overlay exists. Its expected absolute location is
`/home/ros2_ws/install/setup.bash`; existence and successful sourcing are pending.

Matching revisions and clean worktrees identify the deployed baseline but do
not independently prove the host/container bind mount configuration.
The cached runtime-core ref in the development checkout is not listed in the
operator's Pi branch output; do not assume it is available on the Pi.

## Acceptance and next gate

- Host/Docker revision and worktree identification: PASS.
- Basic ROS environment capture: partial; effective RMW remains unverified.
- ROS graph: no nodes discovered; stack status/discovery cause unresolved.
- Map, TF, AMCL, planner lifecycle, Action result, sensors, and QoS: pending.
- Docker build/unit tests and RViz: pending.

Do not start or change the robot stack without reviewing the operator's normal
startup procedure and its possible motion effects. Do not change DDS settings
merely because the graph is empty. Existing navigation-enabled launch files
must not be assumed to be non-actuating.

Phase acceptance: partial, not a completed integration pass.
Phase commit SHA: N/A; no automatic commit authorized.

## Follow-up: approved stack startup and localization blocker

The operator confirmed personnel were physically present and motor power was
off. With motor power kept off, the operator started the standard known-map
bringup inside Docker with explicit `start_waypoints:=false`.
The supplied top-level launch conditions its automatic waypoint node on that
argument. Its waypoint startup log message is unconditional, so that message
alone is not proof of a running goal-sender node.

Workspace overlay sourcing and package discovery succeeded. The installed
map YAML path is a symlink to the source map. The launch readability guard
passed, but this did not establish valid map metadata.

Observed localization result:

```text
map_server: unconfigured [1]
amcl: unconfigured [1]
map_io: Failed processing YAML file .../maps/map_test_1019.yaml ...
Failed to parse YAML tag 'image' ... invalid node; first invalid key: "image"
map_server: Original error: Failed to load map yaml file
lifecycle_manager_localization: Failed to change state for node: map_server
lifecycle_manager_localization: Failed to bring up all requested nodes. Aborting bringup.
```

The planner lifecycle query stalled and was not completed. Navigation lifecycle
startup logs supplied by the operator were partial. Do not infer planner
readiness or its failure cause from this evidence.

Next read-only diagnostic: inspect the actual map YAML and saved map filenames.
Do not fabricate map metadata, modify instructor packages, manually force
lifecycle activation, or run path-planning tests until the map issue is resolved.
Current outcome: localization bringup FAIL; map-content diagnosis pending.

## Confirmed map metadata failure

Subsequent operator-provided read-only inspection established:

- Resolved YAML: `/home/ros2_ws/src/balance_mapping/maps/map_test_1019.yaml`.
- Actual YAML size: **0 bytes**.
- Matching-name PGM size: **28225 bytes**; image validity and identity are not
  established by file size.
- Other nonempty YAML files exist, but none has been verified as matching this
  PGM or the current physical environment.
- `/compute_path_to_pose`: zero Action servers in the supplied diagnostic.
- `/map`: zero publishers in the supplied diagnostic; one transient-local,
  reliable subscriber was reported with unknown node identity.

Root cause of the observed map-server configuration failure: empty map
metadata. This is not evidence of a search-policy implementation defect.
Planning readiness remains FAIL/PENDING, and no planning test was performed.

Recovery requires matching original metadata from an existing backup/history
or another explicitly operator-verified map. Do not infer origin, resolution,
or occupancy thresholds from PGM size or historical map observations. Do not
overwrite the existing files or modify instructor launch/configuration files.
The top-level `map` launch argument can select an approved recovered map later,
without modifying instructor packages. Stack restart and further live testing
remain gated on recovery and operator approval.
