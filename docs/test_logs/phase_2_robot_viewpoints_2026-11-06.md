# Phase 2: robot-map viewpoint checks, 2026-11-06

## Scope and decision

- Classification: automated tests and non-actuating snapshot checks using the
  robot's live `/map`; no planning, navigation or velocity command was sent.
- Date: 2026-11-06. The robot ROS container used UTC; the operator's session
  used UTC+08:00.
- Branch and robot source revision: `robot-validation-hardening` at
  `593ba719958625eeac5884d98e2c7c76b000170e`.
- Robot: Raspberry Pi 5, ROS 2 Humble Docker. The base, Nav2, camera and NCNN
  detector were already running; the search manager was not started.
- Decision: **partial pass only**. Existing unit tests and the snapshot checks
  passed. Map-update safety and a reusable real-map regression fixture remain
  unverified. These results do not authorize robot motion.
- Map: `/map` from the previously verified `map_test_1030` map server; its
  YAML and PGM hashes are recorded in the [Phase 0 audit](phase_0_robot_audit_2026-11-06.md).
  The map files remained untracked on the Pi and were not copied or committed.

## Existing tests

Pi ROS Docker:

```bash
source /home/ros2_ws/install/setup.bash
cd /home/ros2_ws/src/ee3033-search-policy
/usr/bin/python3 -m pytest -q test/test_map_utils.py test/test_runtime_core.py
```

Observed: **34 passed in 0.25 s**. These include synthetic grid geometry,
classification, clearance, line-of-sight and runtime-core tests. They do not
establish that a running search manager safely handles a changed map.

## Live map snapshot and candidate generation

The operator ran the following one-shot subscriber in Pi ROS Docker. It
passed the received `OccupancyGrid` to the NumPy runtime generator and did
not instantiate a search manager or Action client:

```bash
source /home/ros2_ws/install/setup.bash
cd /home/ros2_ws/src/ee3033-search-policy
timeout 90 /usr/bin/python3 - <<'PY'
import time
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from ee3033_search_policy.map_utils import (
    GridGeometry, classify_cells, occupancy_array, world_to_grid,
)
from ee3033_search_policy.runtime_viewpoints import generate_viewpoints_from_grid

rclpy.init()
node = rclpy.create_node('offline_viewpoint_audit')
received = []
qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
subscription = node.create_subscription(OccupancyGrid, '/map', received.append, qos)
deadline = time.monotonic() + 15.0
while not received and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)
if not received:
    raise RuntimeError('No /map received within 15 seconds')

grid = occupancy_array(received[0])
geometry = GridGeometry.from_message(received[0])
free, _, _ = classify_cells(grid)
print('map:', geometry, 'free cells:', int(free.sum()), flush=True)

start = time.monotonic()
result = generate_viewpoints_from_grid(grid, geometry)
elapsed = time.monotonic() - start
assert len(result) > 0 and len(result) % 4 == 0
assert result.visibility_masks.shape == (len(result), *grid.shape)
assert np.all(free[result.rows, result.cols])
assert np.all(result.clearance_m >= 0.20)
assert not np.any(result.visibility_masks & ~free)
assert np.array_equal(result.observable_mask, np.any(result.visibility_masks, axis=0) & free)
np.testing.assert_allclose(
    result.visible_areas_m2,
    result.visibility_masks.sum(axis=(1, 2)) * geometry.resolution ** 2,
    rtol=1e-5, atol=1e-6,
)
for row, col, x, y in zip(result.rows, result.cols, result.world_x, result.world_y):
    assert world_to_grid(float(x), float(y), geometry) == (int(row), int(col))
positions, counts = np.unique(
    np.column_stack((result.rows, result.cols)), axis=0, return_counts=True)
assert np.all(counts == 4)

print('candidates:', len(result), 'positions:', len(positions))
print('minimum clearance (m):', float(result.clearance_m.min()))
print('observable cells:', int(result.observable_mask.sum()))
print('generation time (s):', round(elapsed, 3))
print('real-map checks: PASS')
node.destroy_node()
rclpy.shutdown()
PY
```

Observed: width 78, height 65, resolution approximately 0.05 m, origin
`(-0.881, -1.27, 0)`, 4,390 free cells, 96 oriented candidates at 24
positions, 4,196 observable cells, minimum reported clearance approximately
0.2146 m, and generation time 0.702 s. The stated shape, free-space,
visibility-area and coordinate-roundtrip checks passed for this snapshot.

## Independent geometric clearance check

The operator performed a second one-shot map subscription and regenerated the
positions. The audit independently computed Euclidean distance from each
candidate cell center to the edges of all non-free cell squares and the map
boundary:

```bash
source /home/ros2_ws/install/setup.bash
cd /home/ros2_ws/src/ee3033-search-policy
timeout 90 /usr/bin/python3 - <<'PY'
import time
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from ee3033_search_policy.map_utils import GridGeometry, classify_cells, occupancy_array
from ee3033_search_policy.runtime_viewpoints import generate_viewpoints_from_grid

rclpy.init()
node = rclpy.create_node('clearance_geometry_audit')
maps = []
qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
subscription = node.create_subscription(OccupancyGrid, '/map', maps.append, qos)
deadline = time.monotonic() + 15.0
while not maps and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)
if not maps:
    raise RuntimeError('No /map received within 15 seconds')

grid = occupancy_array(maps[0])
geometry = GridGeometry.from_message(maps[0])
free, _, _ = classify_cells(grid)
blocked_rows, blocked_cols = np.nonzero(~free)
candidates = generate_viewpoints_from_grid(grid, geometry)
positions = np.unique(np.column_stack((candidates.rows, candidates.cols)), axis=0)
minimum = float('inf')
for row, col in positions:
    edge = min(row + 0.5, col + 0.5,
               geometry.height - row - 0.5,
               geometry.width - col - 0.5) * geometry.resolution
    dx = np.maximum(np.abs(blocked_cols - col) - 0.5, 0) * geometry.resolution
    dy = np.maximum(np.abs(blocked_rows - row) - 0.5, 0) * geometry.resolution
    obstacle = np.min(np.hypot(dx, dy)) if len(dx) else float('inf')
    minimum = min(minimum, edge, obstacle)

print('positions checked:', len(positions))
print('minimum geometric clearance (m):', float(minimum))
if minimum < 0.20 - 1e-9:
    raise AssertionError('Candidate clearance is below 0.20 m')
print('geometric clearance check: PASS (provisional 0.20 m only)')
node.destroy_node()
rclpy.shutdown()
PY
```

Observed: 24 positions, minimum geometric clearance
`0.22500000335276127` m; the 0.20 m software threshold passed.
This threshold is provisional and is **not** a verified robot footprint.
The test did not verify physical clearance, reachability, Nav2 paths or
observations.

## Unresolved map-update safety

Source inspection found that the search manager's map callback returns
without retaining a changed map when a decision or navigation is active.
If the publisher does not resend that map, the manager may continue using
the earlier map. This behavior has not been fixed or covered by a map-update
regression test. The operator chose to defer the issue.

Do not treat this partial record as Phase 2 acceptance or enable physical
navigation while the map-update, sensor freshness, TF and other documented
safety blockers remain open. No source changes were made for this phase.

## Provisional camera geometry for later work

The operator confirmed the camera image is upright and the optical axis is
approximately horizontal and forward when the robot is upright. The housing
measurements described in the Phase 0 audit remain estimates, not measured
sensor origins.

Assuming the existing laser TF is accurate, `base_footprint` uses x forward,
y left and z up, and the camera image axes match the standard optical
convention, a provisional camera origin relative to `base_footprint` is
`(-0.018, 0, 0.2375)` m. The corresponding idealized optical rotation maps
optical +x to base -y, optical +y to base -z and optical +z to base +x; its
quaternion `(x, y, z, w)` is `(-0.5, 0.5, -0.5, 0.5)`.

These are offline assumptions only. No `default_cam` transform was published
and no geometry-verification flag was enabled. A static transform does not
account for body pitch. The `/robot_euler` pitch sign, source-sample
freshness, timing relative to each image, rotation pivot and full dynamic TF
remain unverified. These assumptions cannot justify formal target coordinates
or physical navigation.
