#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "Use: source script/unitree_g1_env.sh" >&2
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

UNITREE_ROS2_SETUP="${UNITREE_ROS2_SETUP:-${HOME}/unitree_ros2/setup.sh}"
if [[ -f "${UNITREE_ROS2_SETUP}" ]]; then
  # shellcheck disable=SC1090
  source "${UNITREE_ROS2_SETUP}"
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export OMNIMORPH_ALLOW_ROS_DOMAIN_ID_ZERO="${OMNIMORPH_ALLOW_ROS_DOMAIN_ID_ZERO:-1}"
export OMNIMORPH_SUGGESTED_ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

# shellcheck disable=SC1091
source "${SCRIPT_DIR}/dev_env.sh"
