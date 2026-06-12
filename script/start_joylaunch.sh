#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

cd "${WORKSPACE_DIR}"
source_ros_workspace

prepare_ros_network_env "rmw_fastrtps_cpp" || exit 1
export PYTHONUNBUFFERED=1

exec /usr/bin/python3 "${SCRIPT_DIR}/joyLaunch.py" --workspace "${WORKSPACE_DIR}" "$@"
