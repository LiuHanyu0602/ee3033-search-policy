# EE3033 one-command real-robot workflow

This entry reproduces the successful 2026-11-10/11 physical workflow while
adding operator-controlled SLAM and automatic return to the mapping start.

It owns exactly one mapping/base stack, one known-map Nav2 stack, one camera,
one NCNN detector, and one EE3033 runtime. Stop any older copies first.

## Build once

Inside `ros2_humble`:

```bash
cd /home/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  ee3033_search_policy_interfaces \
  ee3033_search_policy \
  --symlink-install
```

## One command per new run

Choose a new, nonexisting map prefix. Do not include `.yaml` or `.pgm`:

```bash
/home/ros2_ws/src/ee3033-search-policy/scripts/run_real_robot_workflow.sh \
  --map-prefix /home/ros2_ws/src/ee3033-search-policy/maps/map_run_YYYYMMDD_HHMM \
  --navigation-enabled true
```

The command deliberately requires `--navigation-enabled true`. It refuses to
overwrite an existing map.

## Operator interactions

The workflow asks only for these task decisions:

1. press Enter to authorize startup with the floor clear and E-stop ready;
2. manually map with the existing robot controls, complete loop closure, drive
   back near the recorded mapping start, stop issuing manual motion commands,
   then type `DONE`;
3. place the target on the floor outside the initial camera view and type its
   deployed class name, for example `bottle`.

After `DONE`, the workflow automatically:

- records the SLAM-frame pose immediately after mapping becomes ready;
- saves and hashes the new map;
- removes only isolated occupied pixels whose other eight cells in a complete
  3x3 neighborhood are all known free, retaining the original as
  `<map-prefix>.raw.pgm`;
- stops SLAM and starts known-map Nav2 with that map;
- publishes the captured pose to AMCL and verifies localization;
- starts the timestamp-correct 30 Hz camera and `/camera/image_raw` remap;
- starts the external NCNN detector and verifies roughly 5-6 Hz output;
- starts the continuous Tiny-MLP search runtime;
- configures the operator-provided class and begins search;
- waits for confirmed detection, verified Nav2 terminal state, and localization;
- after the search goal reaches its matching terminal state, uses one bounded
  Nav2 `/spin` action to turn toward the calibrated bbox bearing when the
  target is outside the configured central deadband;
- prints the final map coordinate;
- sends one direct Nav2 goal back to the captured mapping start;
- requires remote `SUCCEEDED` before reporting return completion.

The manual controller is external to this workflow. It is neither launched nor
stopped by the script. `DONE` is the operator's assertion that the robot has
returned near the displayed mapping start, is stationary, and the manual
controller is no longer issuing motion commands. A failed start-pose check
keeps SLAM running so the operator can reposition and type `DONE` again.

Detailed child-process output is written under
`/tmp/ee3033_workflow_<timestamp>/`. The console prints phase changes,
readiness, target result, and failures only. If a child exits, its log tail is
shown. A compact persistent `<map-prefix>.run.json` records map hashes, home
pose, target episode, final estimate, return result, or the failure reason.

## Reproduced physical parameters

- repository Tiny MLP and standard search candidate settings;
- camera 640x480 MJPEG-to-RGB at 30 Hz;
- explicit `/image_raw:=/camera/image_raw` remap;
- NCNN detector script `/home/ros2_ws/scripts/yolo_ncnn_node.py`;
- one fresh YOLO frame confirms the target and requests navigation stop;
- detector source continuity age `1.5 s`;
- target observation and localization age remain `0.5 s`;
- detector-only recovery grace `0.75 s`;
- three consistent localization estimates;
- image-stamped LiDAR/camera dual-mode ranging for an object placed on the
  floor: the nearest adjacent-beam support inside the bbox angle sector is
  preferred when it agrees with the calibrated camera ground projection
  within `0.20 m`; otherwise the camera projection is retained. No minimum
  target-to-wall separation is required;
- five valid range observations before a localization candidate is emitted;
- bbox contact with the top or bottom image boundary disables camera ground
  projection for that frame and requires a synchronized LiDAR return;
- the operator console reports alignment state, localization rejection or
  accumulation status, candidate method and final localization method;
- interrupting the one-command workflow during alignment calls
  `/target/cancel_alignment` and waits for the Spin remote terminal state;
- continuous search, at most 100 navigation goals;
- instructor Nav2 controller and costmap parameters are not overridden.

The recovery grace applies only to transient detector status loss. Runtime,
TF, scan, odometry, unknown Nav2 terminal, and process failures remain
fail-closed.

## External paths checked at startup

- timestamp-correct camera overlay:
  `/home/ros2_ws/usb_cam_timestamp_test.hZhIXi/install/usb_cam`;
- camera calibration:
  `/home/ros2_ws/src/ee3033-search-policy/results/updated_calibration/camera_info.yaml`;
- detector script: `/home/ros2_ws/scripts/yolo_ncnn_node.py`;
- camera device: `/dev/video0`.

Missing dependencies or duplicate robot nodes stop the workflow before motion.

## Conservative saved-map cleanup

The cleanup runs after `map_saver_cli` succeeds and before the new map is
loaded by Nav2. It uses `negate`, `occupied_thresh`, and `free_thresh` from the
saved YAML rather than assuming raw black/white values. Unknown cells are not
treated as free. Border cells, walls, connected obstacle pixels, and any cell
with an occupied or unknown neighbor in its 3x3 window are retained.

The workflow explicitly saves with `free_thresh_default=0.196` and validates
the generated YAML before cleanup.  This keeps PGM gray 205 at occupancy
probability `50/255 = 0.196078...` on the unknown side of the strict free
threshold instead of reloading it as free.  A generated YAML containing the
Nav2 CLI default `free_thresh: 0.25` fails closed before cleanup or Nav2 load.

The cleaned PGM remains the YAML's active image. The untouched raster is stored
as `<map-prefix>.raw.pgm`; the run JSON records both hashes and the number of
removed cells. Live Nav2 obstacle layers continue to mark obstacles seen by the
LiDAR even if an isolated static-map speckle was removed.
