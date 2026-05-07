#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${WORKSPACE_DIR}"

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

CAMERA_NAME="${CAMERA_NAME:-camera}"
CAMERA_NAMESPACE="${CAMERA_NAMESPACE:-}"
CAMERA_ROOT="/${CAMERA_NAME}"
if [[ -n "${CAMERA_NAMESPACE}" ]]; then
  CAMERA_ROOT="/${CAMERA_NAMESPACE#/}/${CAMERA_NAME}"
fi

export INPUT_COLOR_TOPIC="${INPUT_COLOR_TOPIC:-${CAMERA_ROOT}/color/image_raw}"
export INPUT_DEPTH_TOPIC="${INPUT_DEPTH_TOPIC:-${CAMERA_ROOT}/depth/image_rect_raw}"
export INPUT_COLOR_INFO_TOPIC="${INPUT_COLOR_INFO_TOPIC:-${CAMERA_ROOT}/color/camera_info}"
export INPUT_DEPTH_INFO_TOPIC="${INPUT_DEPTH_INFO_TOPIC:-${CAMERA_ROOT}/depth/camera_info}"

driver_pid=""

cleanup() {
  if [[ -n "${driver_pid}" ]] && kill -0 "${driver_pid}" >/dev/null 2>&1; then
    kill "${driver_pid}" >/dev/null 2>&1 || true
    wait "${driver_pid}" >/dev/null 2>&1 || true
  fi
}

trap cleanup EXIT INT TERM

"${SCRIPT_DIR}/start_realsense_driver.sh" "$@" &
driver_pid=$!

for _ in $(seq 1 50); do
  if ros2 topic list 2>/dev/null | grep -qx "${INPUT_COLOR_TOPIC}"; then
    break
  fi
  sleep 0.2
done

exec "${SCRIPT_DIR}/start_realsense_bridge.sh"
