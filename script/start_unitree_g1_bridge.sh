#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

print_banner "Unitree G1 Bridge"
source_ros_workspace

BRIDGE_EXEC="$(resolve_ros_executable "unitree_g1_bridge" "unitree_g1_bridge")" || \
  die "unitree_g1_bridge executable not found. Source the official Unitree ROS2 workspace and build package unitree_g1_bridge."

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
log_info "Starting: ${BRIDGE_EXEC} $*"
exec "${BRIDGE_EXEC}" "$@"
