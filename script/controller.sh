#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "[WARN] controller.sh launches the legacy standalone RL_controller process." >&2
echo "[WARN] Standard deploy path no longer needs this process; keep using it only for the Python interactive sim2sim path or isolated debugging." >&2
exec "${SCRIPT_DIR}/run_ros_executable.sh" rl_master RL_controller --system-libstdcxx -- "$@"
