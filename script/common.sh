#!/usr/bin/env bash

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

require_ros_domain_id() {
  local suggested="${OMNIMORPH_SUGGESTED_ROS_DOMAIN_ID:-73}"

  if [[ -z "${ROS_DOMAIN_ID:-}" ]]; then
    log_error "ROS_DOMAIN_ID is not set. Refusing to use ROS 2 default domain 0."
    log_error "Choose a unique domain per robot/test setup, for example:"
    log_error "  export ROS_DOMAIN_ID=${suggested}"
    log_error "Then run the startup command again. Override the suggestion with OMNIMORPH_SUGGESTED_ROS_DOMAIN_ID."
    return 1
  fi

  if ! [[ "${ROS_DOMAIN_ID}" =~ ^[0-9]+$ ]]; then
    log_error "ROS_DOMAIN_ID must be an integer in [1, 232], got: ${ROS_DOMAIN_ID}"
    return 1
  fi

  if (( ROS_DOMAIN_ID < 1 || ROS_DOMAIN_ID > 232 )); then
    if [[ "${ROS_DOMAIN_ID}" == "0" ]]; then
      if [[ "${OMNIMORPH_ALLOW_ROS_DOMAIN_ID_ZERO:-0}" == "1" ]]; then
        log_warn "OMNIMORPH_ALLOW_ROS_DOMAIN_ID_ZERO=1 set; allowing ROS_DOMAIN_ID=0 for this run."
        export ROS_DOMAIN_ID
        return 0
      fi
      log_error "ROS_DOMAIN_ID=0 is blocked by this project because it is the ROS 2 default and can collide on shared LANs."
      log_error "Set a robot-specific domain, e.g. export ROS_DOMAIN_ID=${suggested}."
      log_error "For Unitree G1 ROS2, use: source script/unitree_g1_env.sh"
      log_error "For a one-off lab diagnostic only, set OMNIMORPH_ALLOW_ROS_DOMAIN_ID_ZERO=1."
      return 1
    fi
    log_error "ROS_DOMAIN_ID must be in [1, 232], got: ${ROS_DOMAIN_ID}"
    return 1
  fi

  export ROS_DOMAIN_ID
}

scan_ros_domain_graph() {
  local mode="${OMNIMORPH_ROS_DOMAIN_SCAN:-warn}"
  local timeout_sec="${OMNIMORPH_ROS_DOMAIN_SCAN_TIMEOUT_SEC:-3}"

  case "${mode}" in
    off|false|0)
      return 0
      ;;
    warn|strict)
      ;;
    *)
      log_warn "Unknown OMNIMORPH_ROS_DOMAIN_SCAN=${mode}; use off, warn, or strict. Falling back to warn."
      mode="warn"
      ;;
  esac

  command -v ros2 >/dev/null 2>&1 || {
    log_warn "ros2 command not found; skipping ROS_DOMAIN_ID=${ROS_DOMAIN_ID} graph scan"
    return 0
  }

  command -v timeout >/dev/null 2>&1 || {
    log_warn "timeout command not found; skipping ROS_DOMAIN_ID=${ROS_DOMAIN_ID} graph scan"
    return 0
  }

  local nodes
  nodes="$(timeout "${timeout_sec}" ros2 node list 2>/dev/null || true)"
  nodes="$(printf '%s\n' "${nodes}" | sed '/^[[:space:]]*$/d' || true)"

  if [[ -z "${nodes}" ]]; then
    log_info "ROS_DOMAIN_ID=${ROS_DOMAIN_ID} scan: no existing ROS nodes visible."
    return 0
  fi

  log_warn "ROS_DOMAIN_ID=${ROS_DOMAIN_ID} scan: existing ROS nodes are already visible:"
  printf '%s\n' "${nodes}" | sed 's/^/  /'
  log_warn "If these are not expected local nodes for this robot, another machine may be using the same domain."
  log_warn "Use a different ROS_DOMAIN_ID or set OMNIMORPH_ROS_DOMAIN_SCAN=off after manual verification."

  if [[ "${mode}" == "strict" ]]; then
    log_error "OMNIMORPH_ROS_DOMAIN_SCAN=strict: aborting because the domain is not empty."
    return 1
  fi

  return 0
}

prepare_ros_network_env() {
  local rmw_default="${1:-rmw_fastrtps_cpp}"

  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-${rmw_default}}"
  require_ros_domain_id || return 1
  scan_ros_domain_graph || return 1
}

build_ros_package() {
  local package_name="$1"
  (cd "${WORKSPACE_DIR}" && colcon build --packages-select "${package_name}")
}

detect_unitree_transport_from_config() {
  local cfg_path="$1"
  [[ -f "${cfg_path}" ]] || return 1
  awk -F: '
    /^[[:space:]]*unitree_transport[[:space:]]*:/ {
      value = $2
      sub(/#.*/, "", value)
      gsub(/^[[:space:]]+|[[:space:]]+$/, "", value)
      print value
      exit
    }
  ' "${cfg_path}"
}

is_unitree_ros2_transport() {
  local transport="$1"
  case "${transport}" in
    ros2|dds|unitree_ros2|unitree_g1_dds)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

prepare_unitree_ros2_domain_zero_env() {
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
  export OMNIMORPH_ALLOW_ROS_DOMAIN_ID_ZERO="${OMNIMORPH_ALLOW_ROS_DOMAIN_ID_ZERO:-1}"
  export OMNIMORPH_ROS_DOMAIN_SCAN="${OMNIMORPH_ROS_DOMAIN_SCAN:-warn}"
  log_warn "Unitree ROS2 transport selected; allowing ROS_DOMAIN_ID=${ROS_DOMAIN_ID} to match Unitree's default DDS domain."
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
