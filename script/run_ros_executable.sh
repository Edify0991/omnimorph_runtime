#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

usage() {
  cat <<'EOF'
Usage:
  run_ros_executable.sh <package> <executable> [options] [-- <node args...>]

Options:
  --build-if-missing    Build package when executable is missing.
  --system-libstdcxx    Prepend /usr/lib/aarch64-linux-gnu to LD_LIBRARY_PATH.
  -h, --help            Show this help.
EOF
}

if [[ $# -lt 2 ]]; then
  usage
  exit 2
fi

PACKAGE_NAME="$1"
EXECUTABLE_NAME="$2"
shift 2

BUILD_IF_MISSING=false
USE_SYSTEM_LIBSTDCXX=false
NODE_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build-if-missing)
      BUILD_IF_MISSING=true
      shift
      ;;
    --system-libstdcxx)
      USE_SYSTEM_LIBSTDCXX=true
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

print_banner "ROS2 ${PACKAGE_NAME}/${EXECUTABLE_NAME}"
source_ros_workspace
prepare_ros_network_env "rmw_fastrtps_cpp" || exit 1

EXECUTABLE_PATH=""
if EXECUTABLE_PATH="$(resolve_ros_executable "${PACKAGE_NAME}" "${EXECUTABLE_NAME}")"; then
  :
else
  if [[ "${BUILD_IF_MISSING}" == "true" ]]; then
    log_warn "Executable not found. Building package ${PACKAGE_NAME}..."
    build_ros_package "${PACKAGE_NAME}"
    source_ros_workspace
    prepare_ros_network_env "rmw_fastrtps_cpp" || exit 1
    EXECUTABLE_PATH="$(resolve_ros_executable "${PACKAGE_NAME}" "${EXECUTABLE_NAME}")" || \
      die "Executable ${EXECUTABLE_NAME} still missing after build"
  else
    die "Executable ${EXECUTABLE_NAME} not found. Rebuild package ${PACKAGE_NAME}."
  fi
fi

if [[ "${USE_SYSTEM_LIBSTDCXX}" == "true" ]]; then
  export LD_LIBRARY_PATH="/usr/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH:-}"
fi

log_info "Starting: ${EXECUTABLE_PATH} ${NODE_ARGS[*]:-}"
exec "${EXECUTABLE_PATH}" "${NODE_ARGS[@]}"
