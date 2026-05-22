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

if [[ "$(id -u)" -eq 0 ]]; then
  if [[ -z "${DISPLAY:-}" ]]; then
    export DISPLAY="${DISPLAY:-:0}"
  fi
  if [[ -z "${XAUTHORITY:-}" ]]; then
    if [[ -n "${SUDO_USER:-}" && "${SUDO_USER}" != "root" && -f "/home/${SUDO_USER}/.Xauthority" ]]; then
      export XAUTHORITY="/home/${SUDO_USER}/.Xauthority"
    elif [[ -f "/root/.Xauthority" ]]; then
      export XAUTHORITY="/root/.Xauthority"
    fi
  fi
  export QT_X11_NO_MITSHM="${QT_X11_NO_MITSHM:-1}"
fi

export PYTHONUNBUFFERED=1

exec /usr/bin/python3 -m omnimorph_ops_console.app "$@"
