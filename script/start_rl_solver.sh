#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

print_banner "RL Solver"
source_ros_workspace

SOLVER_EXEC="$(resolve_ros_executable "rl_master" "RL_solver")" || \
  die "RL_solver executable not found"

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
log_info "Starting: ${SOLVER_EXEC} $*"
exec "${SOLVER_EXEC}" "$@"
