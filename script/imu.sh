#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

usage() {
  cat <<'EOF'
Usage: imu.sh [options] [-- <ros args...>]

Options:
  --cleanup-shm         Remove current user's stale shared memory segments before start.
  --start-shm-node      Auto start SharedMemory_node if not running (default).
  --no-start-shm-node   Do not auto start SharedMemory_node.
  --serial-device DEV   Serial device to reset before start (default: /dev/ttyACM0).
  --no-serial-reset     Skip stty serial reset.
  -v, --verbose         Set RCUTILS_LOGGING_LEVEL=DEBUG.
  -h, --help            Show this help.
EOF
}

CLEANUP_SHM=false
START_SHM_NODE=true
SERIAL_DEVICE="/dev/ttyACM0"
RESET_SERIAL=true
NODE_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --cleanup-shm)
      CLEANUP_SHM=true
      shift
      ;;
    --start-shm-node)
      START_SHM_NODE=true
      shift
      ;;
    --no-start-shm-node)
      START_SHM_NODE=false
      shift
      ;;
    --serial-device)
      SERIAL_DEVICE="$2"
      shift 2
      ;;
    --no-serial-reset)
      RESET_SERIAL=false
      shift
      ;;
    -v|--verbose)
      export RCUTILS_LOGGING_LEVEL=DEBUG
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      NODE_ARGS=("$@")
      break
      ;;
    *)
      NODE_ARGS+=("$1")
      shift
      ;;
  esac
done

print_banner "IMU Communication"
source_ros_workspace

if [[ "${CLEANUP_SHM}" == "true" ]]; then
  log_info "Cleaning user shared memory segments"
  cleanup_user_shm
fi

if [[ "${RESET_SERIAL}" == "true" ]]; then
  reset_serial_port "${SERIAL_DEVICE}" "921600"
fi

SHM_PID=""
if [[ "${START_SHM_NODE}" == "true" ]]; then
  if pgrep -f "SharedMemory_node" >/dev/null 2>&1; then
    log_info "SharedMemory_node already running"
  else
    SHM_EXEC="$(resolve_ros_executable "SharedMemory" "SharedMemory_node" 2>/dev/null || true)"
    if [[ -n "${SHM_EXEC}" ]]; then
      log_info "Starting SharedMemory_node in background"
      "${SHM_EXEC}" >/tmp/SharedMemory_node.log 2>&1 &
      SHM_PID="$!"
      sleep 1
    else
      log_warn "SharedMemory_node not found. Build package SharedMemory if required."
    fi
  fi
fi

cleanup() {
  if [[ -n "${SHM_PID}" ]]; then
    log_info "Stopping SharedMemory_node (pid=${SHM_PID})"
    kill "${SHM_PID}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

IMU_EXEC="$(resolve_ros_executable "imu_communication_yesense" "imu_communication_yesense")" || \
  die "imu_communication_yesense executable not found"

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
log_info "Starting: ${IMU_EXEC} ${NODE_ARGS[*]:-}"
exec "${IMU_EXEC}" "${NODE_ARGS[@]}"

