#!/usr/bin/env bash

set -euo pipefail

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "[error] missing command: $1" >&2
    exit 1
  fi
}

SESSION="${SESSION:-obs_avoid_mapping}"
USE_TMUX="${USE_TMUX:-0}"
RECREATE_SESSION="${RECREATE_SESSION:-1}"
KILL_BEFORE_LAUNCH="${KILL_BEFORE_LAUNCH:-1}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS_DEFAULT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
ROS_WS="${ROS_WS:-${ROS_WS_DEFAULT}}"
ROS_SETUP="${ROS_SETUP:-${ROS_WS}/install/setup.bash}"

SLAM_PARAMS_FILE="${SLAM_PARAMS_FILE:-${ROS_WS}/src/uav_stack_bringup/config/slam_mapping_fast.yaml}"
MAPPER_PARAMS_FILE="${MAPPER_PARAMS_FILE:-${ROS_WS}/src/vertical_lidar_mapper/config/params.yaml}"
SCAN_TOPIC="${SCAN_TOPIC:-/scan_vertical}"
TARGET_FRAME="${TARGET_FRAME:-map}"
USE_SIM_TIME="${USE_SIM_TIME:-true}"

kill_if_running() {
  local pattern="$1"
  pgrep -af "${pattern}" >/dev/null 2>&1 || return 0
  pkill -f "${pattern}" >/dev/null 2>&1 || true
}

kill_existing_mapping() {
  kill_if_running "ros2 launch slam_toolbox online_async_launch.py"
  kill_if_running "async_slam_toolbox_node"
  kill_if_running "sync_slam_toolbox_node"
  kill_if_running "ros2 launch vertical_lidar_mapper vertical_lidar_mapper.launch.py"
  kill_if_running "vertical_lidar_mapper_node"
}

json_escape() {
  printf '%s' "$1" | sed -e 's/\\/\\\\/g' -e 's/"/\\"/g'
}

require_cmd ros2

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "[error] ROS setup not found: ${ROS_SETUP}" >&2
  exit 1
fi
if [[ ! -f "${SLAM_PARAMS_FILE}" ]]; then
  echo "[error] slam params not found: ${SLAM_PARAMS_FILE}" >&2
  exit 1
fi
if [[ ! -f "${MAPPER_PARAMS_FILE}" ]]; then
  echo "[error] mapper params not found: ${MAPPER_PARAMS_FILE}" >&2
  exit 1
fi

if [[ "${KILL_BEFORE_LAUNCH}" == "1" ]]; then
  kill_existing_mapping
fi

cmd_slam="source '${ROS_SETUP}' && export COLCON_TRACE=\${COLCON_TRACE:-0} && echo '[wait] waiting for /clock /scan_horizontal /mavros/local_position/odom ...' && until ros2 topic list 2>/dev/null | grep -q '^/clock$'; do sleep 1; done && until ros2 topic list 2>/dev/null | grep -q '^/scan_horizontal$'; do sleep 1; done && until ros2 topic list 2>/dev/null | grep -q '^/mavros/local_position/odom$'; do sleep 1; done && echo '[wait] waiting for odom TF source node ...' && until ros2 node list 2>/dev/null | grep -Eq '^/px4_odom_flatten_node$|^/odom_to_tf_bridge$'; do sleep 1; done && echo '[run] starting slam_toolbox' && ros2 launch slam_toolbox online_async_launch.py slam_params_file:='${SLAM_PARAMS_FILE}' use_sim_time:='${USE_SIM_TIME}'"
cmd_mapper="source '${ROS_SETUP}' && export COLCON_TRACE=\${COLCON_TRACE:-0} && echo '[wait] waiting for /scan_vertical and /slam_toolbox ...' && until ros2 topic list 2>/dev/null | grep -q '^/scan_vertical$'; do sleep 1; done && until ros2 node list 2>/dev/null | grep -q '^/slam_toolbox$'; do sleep 1; done && if [[ '${TARGET_FRAME}' == 'map' ]]; then echo '[wait] waiting for /map ...'; until ros2 topic list 2>/dev/null | grep -q '^/map$'; do sleep 1; done; sleep 2; fi && echo '[run] starting vertical_lidar_mapper' && ros2 launch vertical_lidar_mapper vertical_lidar_mapper.launch.py params_file:='${MAPPER_PARAMS_FILE}' scan_topic:='${SCAN_TOPIC}' target_frame:='${TARGET_FRAME}' enable_odom_tf_bridge:=false use_sim_time:='${USE_SIM_TIME}'"

if [[ "${USE_TMUX}" != "1" ]] || ! command -v tmux >/dev/null 2>&1; then
  if ! command -v terminator >/dev/null 2>&1; then
    echo "[error] missing command: terminator" >&2
    exit 1
  fi

  LAYOUT_JSON="${LAYOUT_JSON:-/tmp/obs_avoid_mapping_layout.json}"
  cat > "${LAYOUT_JSON}" <<EOF
{
  "profile": {
    "exit_action": "hold"
  },
  "layout": {
    "vertical": true,
    "Main": [
      {
        "children": [
          { "command": "$(json_escape "${cmd_slam}")" },
          { "command": "$(json_escape "${cmd_mapper}")" }
        ]
      }
    ]
  }
}
EOF
  exec terminator -j "${LAYOUT_JSON}"
fi

if tmux has-session -t "${SESSION}" 2>/dev/null && [[ "${RECREATE_SESSION}" == "1" ]]; then
  tmux kill-session -t "${SESSION}"
fi

if ! tmux has-session -t "${SESSION}" 2>/dev/null; then
  tmux new-session -d -s "${SESSION}" -n mapping

  tmux send-keys -t "${SESSION}:mapping.0" "${cmd_slam}" C-m
  tmux split-window -h -t "${SESSION}:mapping.0"
  tmux send-keys -t "${SESSION}:mapping.1" "${cmd_mapper}" C-m
  tmux select-layout -t "${SESSION}:mapping" tiled
fi

if command -v terminator >/dev/null 2>&1; then
  exec terminator -x bash -lc "tmux attach -t '${SESSION}'"
fi

echo "[warn] terminator not found, attaching to tmux in current terminal."
exec tmux attach -t "${SESSION}"

