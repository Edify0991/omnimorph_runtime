#!/usr/bin/env bash
set -euo pipefail

COMMON_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${COMMON_DIR}/.." && pwd)"

if [[ -t 1 ]]; then
  COLOR_RED='\033[0;31m'
  COLOR_GREEN='\033[0;32m'
  COLOR_YELLOW='\033[1;33m'
  COLOR_BLUE='\033[0;34m'
  COLOR_RESET='\033[0m'
else
  COLOR_RED=''
  COLOR_GREEN=''
  COLOR_YELLOW=''
  COLOR_BLUE=''
  COLOR_RESET=''
fi

log_info() {
  echo -e "${COLOR_GREEN}[INFO]${COLOR_RESET} $*"
}

log_warn() {
  echo -e "${COLOR_YELLOW}[WARN]${COLOR_RESET} $*"
}

log_error() {
  echo -e "${COLOR_RED}[ERROR]${COLOR_RESET} $*" >&2
}

die() {
  log_error "$*"
  exit 1
}

print_banner() {
  local name="$1"
  echo -e "${COLOR_BLUE}=== ${name} ===${COLOR_RESET}"
  echo -e "${COLOR_YELLOW}Workspace: ${WORKSPACE_DIR}${COLOR_RESET}"
}

_detect_ros_setup() {
  local candidates=()
  if [[ -n "${ROS_DISTRO:-}" ]]; then
    candidates+=("/opt/ros/${ROS_DISTRO}/setup.bash")
  fi

  candidates+=(
    "/opt/ros/humble/setup.bash"
    "/opt/ros/jazzy/setup.bash"
    "/opt/ros/iron/setup.bash"
    "/opt/ros/galactic/setup.bash"
    "/opt/ros/foxy/setup.bash"
  )

  local setup_path
  for setup_path in "${candidates[@]}"; do
    if [[ -f "${setup_path}" ]]; then
      echo "${setup_path}"
      return 0
    fi
  done
  return 1
}

source_ros_workspace() {
  local ros_setup
  ros_setup="$(_detect_ros_setup)" || die "ROS2 setup.bash not found under /opt/ros"

  local restore_nounset=0
  if [[ -o nounset ]]; then
    restore_nounset=1
    set +u
  fi

  # shellcheck disable=SC1090
  source "${ros_setup}"

  local ws_setup="${WORKSPACE_DIR}/install/setup.bash"
  [[ -f "${ws_setup}" ]] || die "Workspace is not built. Run: colcon build"

  # shellcheck disable=SC1090
  source "${ws_setup}"

  if [[ "${restore_nounset}" -eq 1 ]]; then
    set -u
  fi
}

build_ros_package() {
  local package_name="$1"
  (cd "${WORKSPACE_DIR}" && colcon build --packages-select "${package_name}")
}

resolve_ros_executable() {
  local package_name="$1"
  local executable_name="$2"

  local preferred="${WORKSPACE_DIR}/install/${package_name}/lib/${package_name}/${executable_name}"
  if [[ -x "${preferred}" ]]; then
    echo "${preferred}"
    return 0
  fi

  local found=""
  found="$(find "${WORKSPACE_DIR}/install" "${WORKSPACE_DIR}/build" \
    -type f -name "${executable_name}" -path "*/${package_name}/*" 2>/dev/null | head -n 1 || true)"

  if [[ -n "${found}" && -x "${found}" ]]; then
    echo "${found}"
    return 0
  fi

  return 1
}

reset_serial_port() {
  local device="$1"
  local baudrate="${2:-921600}"

  if [[ ! -c "${device}" ]]; then
    log_warn "Serial device ${device} does not exist, skip reset"
    return 0
  fi

  stty -F "${device}" "${baudrate}" raw -echo -crtscts -ixon >/dev/null 2>&1 || \
    log_warn "Failed to reset serial device ${device}"
}
