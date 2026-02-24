#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APP_DIR="${SCRIPT_DIR}/pc_env_viewer"
APP_BIN="${APP_DIR}/build/pc_env_viewer"
DEFAULT_FILE="/tmp/vertical_mapper_exports/vertical_global_map_379_368000000.pcd"

if [[ ! -x "${APP_BIN}" ]]; then
  echo "[info] app binary not found, building..."
  cmake --build "${APP_DIR}/build" -j4
fi

if [[ $# -gt 0 ]]; then
  FILE_PATH="$1"
else
  FILE_PATH="${DEFAULT_FILE}"
fi

if [[ -f "${FILE_PATH}" ]]; then
  exec "${APP_BIN}" --file "${FILE_PATH}"
else
  echo "[warn] file not found: ${FILE_PATH}"
  echo "[info] launching app without auto-load"
  exec "${APP_BIN}"
fi
