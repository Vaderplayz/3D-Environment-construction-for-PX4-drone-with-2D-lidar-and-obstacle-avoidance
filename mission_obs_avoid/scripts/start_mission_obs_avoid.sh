#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS_DEFAULT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
ROS_WS="${ROS_WS:-${ROS_WS_DEFAULT}}"
ROS_SETUP="${ROS_SETUP:-${ROS_WS}/install/setup.bash}"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "[error] ROS setup not found: ${ROS_SETUP}" >&2
  exit 1
fi

set +u
source "${ROS_SETUP}"
set -u

if ! ros2 pkg prefix mission_obs_avoid >/dev/null 2>&1; then
  echo "[error] ROS package 'mission_obs_avoid' not found in current overlay." >&2
  echo "[hint] build with:" >&2
  echo "       COLCON_LOG_PATH=/tmp/colcon_log_mission \\" >&2
  echo "       colcon build --base-paths ${ROS_WS}/src/obs_avoid/mission_obs_avoid \\" >&2
  echo "         --packages-select mission_obs_avoid" >&2
  exit 1
fi

exec ros2 launch mission_obs_avoid mission_obs_avoid.launch.py use_sim_time:=true "$@"
