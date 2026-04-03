#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

usage() {
  cat <<'EOF'
Usage: imu.sh [options] [-- <ros args...>]

Options:
  --serial-device DEV   Serial device to reset before start (default: /dev/ttyACM0).
  --no-serial-reset     Skip stty serial reset.
  -v, --verbose         Set RCUTILS_LOGGING_LEVEL=DEBUG.
  -h, --help            Show this help.
EOF
}

SERIAL_DEVICE="/dev/ttyACM0"
RESET_SERIAL=true
NODE_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
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

if [[ "${RESET_SERIAL}" == "true" ]]; then
  reset_serial_port "${SERIAL_DEVICE}" "921600"
fi

IMU_EXEC="$(resolve_ros_executable "imu_communication_yesense" "imu_communication_yesense")" || \
  die "imu_communication_yesense executable not found"

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
log_info "Starting: ${IMU_EXEC} ${NODE_ARGS[*]:-}"
exec "${IMU_EXEC}" "${NODE_ARGS[@]}"
