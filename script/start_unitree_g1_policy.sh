#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

RL_CFG="${WORKSPACE_DIR}/src/omnimorph_rl_controller/rl_master/config/rl_cfg_unitree_g1.yaml"

exec "${SCRIPT_DIR}/sim2real_runtime.sh" --rl-cfg "${RL_CFG}" "$@"
