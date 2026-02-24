#!/usr/bin/env bash

set -euo pipefail

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "[error] missing command: $1" >&2
    exit 1
  fi
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS_DEFAULT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
ROS_WS="${ROS_WS:-${ROS_WS_DEFAULT}}"
ROS_SETUP="${ROS_SETUP:-${ROS_WS}/install/setup.bash}"

SLAM_PARAMS_FILE="${SLAM_PARAMS_FILE:-${ROS_WS}/src/uav_stack_bringup/config/slam_mapping_fast.yaml}"
MAPPER_PARAMS_FILE="${MAPPER_PARAMS_FILE:-${ROS_WS}/src/vertical_lidar_mapper/config/params.yaml}"
SCAN_TOPIC="${SCAN_TOPIC:-/scan_vertical}"
TARGET_FRAME="${TARGET_FRAME:-map}"
USE_SIM_TIME="${USE_SIM_TIME:-true}"
ENABLE_SPIRAL="${ENABLE_SPIRAL:-true}"
ENABLE_DODGE_PLANNER="${ENABLE_DODGE_PLANNER:-true}"
PLANNER_NODE="${PLANNER_NODE:-local_planner_mode_a}"
AUTO_SCAN_RELAY_FOR_SCAN_TOPIC="${AUTO_SCAN_RELAY_FOR_SCAN_TOPIC:-true}"

WAIT_TIMEOUT_SEC="${WAIT_TIMEOUT_SEC:-60}"
WAIT_TOPICS=(
  "/clock"
  "/scan_horizontal"
  "/scan_vertical"
  "/mavros/local_position/odom"
)
WAIT_TF_SOURCE_NODES=(
  "/px4_odom_flatten_node"
  "/odom_to_tf_bridge"
)

SLAM_LOG="${SLAM_LOG:-/tmp/slam_mapping_fast.log}"
MAPPER_LOG="${MAPPER_LOG:-/tmp/vertical_lidar_mapper.log}"
SPIRAL_LOG="${SPIRAL_LOG:-/tmp/spiral_mapping_mode.log}"
PLANNER_LOG="${PLANNER_LOG:-/tmp/obs_avoid_planner.log}"
SCAN_RELAY_LOG="${SCAN_RELAY_LOG:-/tmp/obs_avoid_scan_relay.log}"

SLAM_PID=""
MAPPER_PID=""
SPIRAL_PID=""
PLANNER_PID=""
SCAN_RELAY_PID=""

is_true() {
  case "${1,,}" in
    1|true|yes|on) return 0 ;;
    *) return 1 ;;
  esac
}

cleanup() {
  set +e
  if [[ -n "${SCAN_RELAY_PID}" ]] && kill -0 "${SCAN_RELAY_PID}" >/dev/null 2>&1; then
    echo "[stop] scan relay (pid=${SCAN_RELAY_PID})"
    kill "${SCAN_RELAY_PID}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${PLANNER_PID}" ]] && kill -0 "${PLANNER_PID}" >/dev/null 2>&1; then
    echo "[stop] planner (pid=${PLANNER_PID})"
    kill "${PLANNER_PID}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${SPIRAL_PID}" ]] && kill -0 "${SPIRAL_PID}" >/dev/null 2>&1; then
    echo "[stop] spiral_mapping_mode (pid=${SPIRAL_PID})"
    kill "${SPIRAL_PID}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${MAPPER_PID}" ]] && kill -0 "${MAPPER_PID}" >/dev/null 2>&1; then
    echo "[stop] vertical_lidar_mapper (pid=${MAPPER_PID})"
    kill "${MAPPER_PID}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${SLAM_PID}" ]] && kill -0 "${SLAM_PID}" >/dev/null 2>&1; then
    echo "[stop] slam_toolbox (pid=${SLAM_PID})"
    kill "${SLAM_PID}" >/dev/null 2>&1 || true
  fi
}

wait_for_topic() {
  local topic="$1"
  local timeout_sec="$2"
  local start_ts
  start_ts="$(date +%s)"

  while true; do
    if ros2 topic list 2>/dev/null | grep -qx "${topic}"; then
      return 0
    fi
    if (( "$(date +%s)" - start_ts >= timeout_sec )); then
      echo "[error] timed out waiting for topic: ${topic}" >&2
      return 1
    fi
    sleep 1
  done
}

wait_for_required_topics() {
  local topic
  for topic in "${WAIT_TOPICS[@]}"; do
    echo "[wait] topic ${topic}"
    wait_for_topic "${topic}" "${WAIT_TIMEOUT_SEC}"
  done
}

wait_for_tf_source_node() {
  local start_ts
  start_ts="$(date +%s)"

  while true; do
    local node_list
    node_list="$(ros2 node list 2>/dev/null || true)"
    local node
    for node in "${WAIT_TF_SOURCE_NODES[@]}"; do
      if printf '%s\n' "${node_list}" | grep -qx "${node}"; then
        echo "[wait] found TF source node: ${node}"
        return 0
      fi
    done

    if (( "$(date +%s)" - start_ts >= WAIT_TIMEOUT_SEC )); then
      echo "[error] timed out waiting for TF source node (${WAIT_TF_SOURCE_NODES[*]})" >&2
      return 1
    fi
    sleep 1
  done
}

main() {
  require_cmd ros2

  case "${PLANNER_NODE}" in
    local_planner_mode_a|local_planner_sector_mode|local_planner_hybrid_mode) ;;
    *)
      echo "[error] invalid PLANNER_NODE='${PLANNER_NODE}'" >&2
      echo "        allowed: local_planner_mode_a | local_planner_sector_mode | local_planner_hybrid_mode" >&2
      exit 1
      ;;
  esac

  if [[ ! -f "${ROS_SETUP}" ]]; then
    echo "[error] ROS setup not found: ${ROS_SETUP}" >&2
    exit 1
  fi
  if [[ ! -f "${SLAM_PARAMS_FILE}" ]]; then
    echo "[error] SLAM params file not found: ${SLAM_PARAMS_FILE}" >&2
    exit 1
  fi
  if [[ ! -f "${MAPPER_PARAMS_FILE}" ]]; then
    echo "[error] mapper params file not found: ${MAPPER_PARAMS_FILE}" >&2
    exit 1
  fi

  # setup.bash may reference COLCON_TRACE while unset; temporarily disable nounset.
  set +u
  source "${ROS_SETUP}"
  set -u
  export COLCON_TRACE="${COLCON_TRACE:-0}"

  trap cleanup EXIT INT TERM

  wait_for_required_topics
  wait_for_tf_source_node

  if is_true "${AUTO_SCAN_RELAY_FOR_SCAN_TOPIC}" && [[ "${PLANNER_NODE}" != "local_planner_mode_a" ]]; then
    if ! ros2 topic list 2>/dev/null | grep -qx "/scan"; then
      echo "[run] topic relay /scan_horizontal -> /scan -> ${SCAN_RELAY_LOG}"
      ros2 run topic_tools relay /scan_horizontal /scan >"${SCAN_RELAY_LOG}" 2>&1 &
      SCAN_RELAY_PID="$!"
      sleep 1
    fi
  fi

  if is_true "${ENABLE_DODGE_PLANNER}"; then
    echo "[run] ${PLANNER_NODE} -> ${PLANNER_LOG}"
    ros2 run obs_avoid "${PLANNER_NODE}" --ros-args -p use_sim_time:="${USE_SIM_TIME}" \
      >"${PLANNER_LOG}" 2>&1 &
    PLANNER_PID="$!"
  fi

  if is_true "${ENABLE_SPIRAL}"; then
    echo "[run] spiral_mapping_mode -> ${SPIRAL_LOG}"
    ros2 run obs_avoid spiral_mapping_mode --ros-args -p use_sim_time:="${USE_SIM_TIME}" \
      >"${SPIRAL_LOG}" 2>&1 &
    SPIRAL_PID="$!"
  fi

  echo "[run] slam_toolbox -> ${SLAM_LOG}"
  ros2 launch slam_toolbox online_async_launch.py \
    slam_params_file:="${SLAM_PARAMS_FILE}" \
    use_sim_time:="${USE_SIM_TIME}" >"${SLAM_LOG}" 2>&1 &
  SLAM_PID="$!"

  sleep 3

  echo "[run] vertical_lidar_mapper -> ${MAPPER_LOG}"
  ros2 launch vertical_lidar_mapper vertical_lidar_mapper.launch.py \
    params_file:="${MAPPER_PARAMS_FILE}" \
    scan_topic:="${SCAN_TOPIC}" \
    target_frame:="${TARGET_FRAME}" \
    use_sim_time:="${USE_SIM_TIME}" >"${MAPPER_LOG}" 2>&1 &
  MAPPER_PID="$!"

  echo "[ok] mapping mode started"
  if [[ -n "${SPIRAL_PID}" ]]; then
    echo "[info] spiral pid=${SPIRAL_PID}"
    echo "[info] tail -f ${SPIRAL_LOG}"
  fi
  if [[ -n "${PLANNER_PID}" ]]; then
    echo "[info] planner pid=${PLANNER_PID} (${PLANNER_NODE})"
    echo "[info] tail -f ${PLANNER_LOG}"
  fi
  if [[ -n "${SCAN_RELAY_PID}" ]]; then
    echo "[info] scan relay pid=${SCAN_RELAY_PID}"
    echo "[info] tail -f ${SCAN_RELAY_LOG}"
  fi
  echo "[info] slam pid=${SLAM_PID}, mapper pid=${MAPPER_PID}"
  echo "[info] tail -f ${SLAM_LOG}"
  echo "[info] tail -f ${MAPPER_LOG}"

  local pids=("${SLAM_PID}" "${MAPPER_PID}")
  if [[ -n "${SPIRAL_PID}" ]]; then pids+=("${SPIRAL_PID}"); fi
  if [[ -n "${PLANNER_PID}" ]]; then pids+=("${PLANNER_PID}"); fi
  if [[ -n "${SCAN_RELAY_PID}" ]]; then pids+=("${SCAN_RELAY_PID}"); fi

  wait -n "${pids[@]}"
  exit_code=$?
  echo "[warn] one process exited (code=${exit_code}), stopping the other."
  exit "${exit_code}"
}

main "$@"
