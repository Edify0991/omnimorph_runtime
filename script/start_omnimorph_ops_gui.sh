#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

cd "${WORKSPACE_DIR}"
source_ros_workspace

prepare_ros_network_env "rmw_fastrtps_cpp" || exit 1

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
