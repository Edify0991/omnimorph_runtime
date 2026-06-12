#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

print_banner "RL Solver"

detect_rl_cfg_arg() {
  local next_is_path=0
  local arg
  for arg in "$@"; do
    if [[ "${next_is_path}" -eq 1 ]]; then
      echo "${arg}"
      return 0
    fi
    case "${arg}" in
      --rl-cfg|--config)
        next_is_path=1
        ;;
      --rl-cfg=*|--config=*)
        echo "${arg#*=}"
        return 0
        ;;
    esac
  done
  return 1
}

detect_unitree_transport() {
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

RL_CFG_ARG="$(detect_rl_cfg_arg "$@" || true)"
RMW_DEFAULT="rmw_fastrtps_cpp"
if [[ -n "${RL_CFG_ARG}" ]]; then
  case "$(basename "${RL_CFG_ARG}")" in
    rl_cfg_unitree_g1.yaml)
      UNITREE_TRANSPORT="$(detect_unitree_transport "${RL_CFG_ARG}" || true)"
      case "${UNITREE_TRANSPORT}" in
        ros2|dds|unitree_ros2|unitree_g1_dds)
          RMW_DEFAULT="rmw_cyclonedds_cpp"
          ;;
        *)
          RMW_DEFAULT="rmw_fastrtps_cpp"
          ;;
      esac
      ;;
  esac
fi

source_ros_workspace

SOLVER_EXEC="$(resolve_ros_executable "rl_master" "RL_solver")" || \
  die "RL_solver executable not found"

prepare_ros_network_env "${RMW_DEFAULT}" || exit 1
if [[ "${RMW_IMPLEMENTATION}" == "rmw_cyclonedds_cpp" && -z "${CYCLONEDDS_URI:-}" ]]; then
  log_warn "RMW is rmw_cyclonedds_cpp but CYCLONEDDS_URI is empty. For Unitree, source unitree_ros2/setup.sh first."
fi
log_info "Starting: ${SOLVER_EXEC} $*"
exec "${SOLVER_EXEC}" "$@"
