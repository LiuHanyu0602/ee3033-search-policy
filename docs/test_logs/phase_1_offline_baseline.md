# Phase 1: existing local baseline tests

## Scope and environment

- Date: 2026-10-19, timezone +08:00.
- Classification: local unit tests and static checks on FAGIO; not ROS
  integration, rosbag replay, or physical navigation.
- Branch: `nav2-interface`, retained at the operator's explicit request.
- Revision: `9abc1ada1eefea5897e553a8be818110874f13de`.
- Python 3.12.3, NumPy 2.5.4, pytest 9.1.1 in the existing session-local
  virtual environment. No dependencies were installed for this phase.
- Purpose: reproduce the active checkout's baseline before runtime edits.
- Runtime source changes: none.

## Commands and observed results

Terminal: FAGIO development shell, repository root.

`$PYTHON` below denotes the existing session-local virtual environment's Python;
it is not the ROS Docker Python.

```bash
"$PYTHON" -m pytest -q test
python3 -m compileall -q ee3033_search_policy launch test
git diff --check
```

Observed test output:

```text
.........................................................                [100%]
57 passed in 0.16s
```

Compilation and whitespace checks exited successfully with no error output.
The Action and planner-node tests use test doubles; they do not exercise DDS
or actual ROS message serialization.

## Acceptance and limitations

- Existing local suite: PASS (57 tests, no failures).
- Syntax/whitespace: PASS.
- ROS Humble Python 3.10 execution: pending.
- Docker build/colcon tests: pending; robot disconnected.
- Current-pose TF and real Nav2 planning: pending; robot disconnected.
- Remote RViz: pending.
- Runtime-core suite: not run; those source/test files are present only in the
  cached `origin/runtime-policy-core` ref, not in this checkout.
- Candidate count, map signature, planning time, request latency, path length,
  policy inference time, CPU/memory, and physical outcomes: N/A, not measured.

Offline baseline passes. Full robot-side Phase 1 acceptance remains pending.
The information gaps in [Phase 0](phase_0_offline_audit.md) remain open.
Next-phase implementation requires operator acceptance; no automatic commit
is authorized. Phase commit SHA: N/A.

## 2026-10-29: approved integration-branch baseline

- Classification: offline unit tests, model reference comparison, and syntax
  checks on the development machine; not ROS integration or physical motion.
- Branch: `robot-runtime-integration`, created with operator approval from
  `origin/return-to-origin-integration`.
- Baseline revision: `cd3daca3ea5ee52ee54dba6f47c54102f91ac6cd`.
- The operator approved an isolated session-local test environment. System
  Python and repository dependency manifests were not changed.
- Environment: Python 3.12, NumPy 2.5.4, pytest 9.2.0. ROS and colcon are not
  available in this development shell.
- Runtime source changes before these tests: none.

Development-machine terminal, repository root, using the approved isolated
environment's Python as `$PYTHON`:

```bash
"$PYTHON" -m pytest test -q
(cd models && sha256sum -c SHA256SUMS)
"$PYTHON" scripts/numpy_search_policy.py \
  --check-reference models/tiny_search_policy_reference.npz --iterations 1000
```

Observed results:

- Unit tests: **69 passed in 0.41 s**.
- Frozen weights SHA256: PASS.
- Standalone NumPy reference comparison: PASS; maximum absolute error
  `7.153e-07`, with `rtol=1e-5` and `atol=1e-5`.
- Standalone inference for 32 candidates: mean `0.021032 ms`, P95
  `0.031950 ms` over 1,000 iterations on this development machine. These are
  not Raspberry Pi or full-decision latency measurements.
- AST syntax checks: PASS for 20 package, launch, and setup Python files.
- Operator accepted the offline branch-preparation checkpoint and authorized
  a separate offline readiness-safeguard integration step.

No robot stack, DDS configuration, navigation goal, or velocity command was
started or changed. Live map, TF, localization, planner, sensors, ROS Humble
build, and physical safety checks remain pending. The historical robot audit's
empty map YAML is unresolved by these offline results; verify its current
state before robot-side planning.

## 2026-10-29: offline readiness-safeguard integration

- Classification: offline unit tests and static checks; no live ROS or motion.
- Branch: `robot-runtime-integration`.
- Parent checkpoint: `e95e72a` (guidance and baseline records).
- Source intent: divergent readiness fix `81df4d9`, adapted rather than merged
  wholesale into the return-to-origin implementation.
- Map bounds validation reuses `world_to_grid` in `_robot_pose`, protecting both
  planning and mission-start home capture. Existing TF/planner wait states and
  rotated-map geometry are preserved.
- Planner goal rejection stops the batch without blacklisting candidates and
  delays decision retry for five monotonic seconds. Return-home handling and
  observation completion retain priority. Mission/memory reset clears backoff.
- Guidance now names the existing `navigation_enabled` motion gate rather than
  an unsupported `enable_motion` alias. Motion defaults were not changed.

Using the same approved isolated development environment:

```bash
"$PYTHON" -m pytest test -q
git diff --check
```

Observed results:

- **86 passed in 0.29 s**, including 17 new readiness regression cases.
- Covered map boundaries, non-finite position, rotated origin, invalid home
  capture, missing map/TF/planner, exact five-second retry boundary, rejected
  versus accepted planning outcomes, return/observation priority, and mission
  reset.
- AST syntax checks: PASS for 25 package, launch, test, and setup Python files.
- Whitespace checks: PASS.

No robot-side dependencies were installed and no launch or motion command was
run. TF freshness, actual planner lifecycle, ROS build, and on-robot acceptance
remain pending. After reviewing these results, the operator approved the
offline runtime checkpoint and its local commit. This does not authorize
robot motion or establish live integration acceptance.
