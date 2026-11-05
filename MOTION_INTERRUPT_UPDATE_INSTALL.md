# Install and verify the motion-interrupt update

Apply this archive only on `robot-validation-hardening` after confirming a clean
tracked worktree.  Untracked report assets may remain.

```bash
cd ~/ee3033_ws/src/ee3033_search_policy
git branch --show-current
git status --short
```

Extract the archive from Downloads:

```bash
tar -xzf ~/Downloads/ee3033_motion_interrupt_localization_update.tar.gz \
  -C ~/ee3033_ws/src/ee3033_search_policy
```

Run offline verification in the ROS Python 3.10 environment:

```bash
source ~/start_ee3033.sh
cd ~/ee3033_ws/src/ee3033_search_policy

/usr/bin/python3 -m compileall -q ee3033_search_policy launch test
/usr/bin/python3 -m pytest test/test_balance_observation_gate.py -q
/usr/bin/python3 -m pytest test/test_complete_mission_protocol.py -q
/usr/bin/python3 -m pytest test -q
git diff --check
```

Then build both packages:

```bash
cd ~/ee3033_ws
colcon build --packages-select \
  ee3033_search_policy_interfaces \
  ee3033_search_policy \
  --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

Review before committing:

```bash
cd ~/ee3033_ws/src/ee3033_search_policy
git diff --stat
git diff --check
git status --short
```

Do not add `report_assets/` or `config/yolo_labels_TEST_ONLY.json` unless the
team explicitly intends to version those existing untracked files.
