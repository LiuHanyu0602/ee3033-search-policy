#!/usr/bin/env bash
# ROS 2 Humble setup files are not safe under nounset: they intentionally read
# AMENT_TRACE_SETUP_FILES before defining it. Enable nounset only after all
# environment overlays have been sourced.
set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/ros2_ws/install/setup.bash

camera_overlay=/home/ros2_ws/usb_cam_timestamp_test.hZhIXi/install/setup.bash
if [[ ! -f "$camera_overlay" ]]; then
  echo "STOP: timestamp-correct camera overlay is missing: $camera_overlay" >&2
  exit 1
fi
source "$camera_overlay"

set -u

export ROS_DOMAIN_ID=15
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_LOCALHOST_ONLY || true

exec python3 -m ee3033_search_policy.real_robot_workflow "$@"
