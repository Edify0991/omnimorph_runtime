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
export PYTHONUNBUFFERED=1

exec /usr/bin/python3 "${SCRIPT_DIR}/joyLaunch.py" --workspace "${WORKSPACE_DIR}" "$@"
