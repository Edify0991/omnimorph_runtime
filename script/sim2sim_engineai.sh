#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

log_warn "sim2sim_engineai.sh is deprecated; use sim2sim_runtime.sh instead."
exec "${SCRIPT_DIR}/sim2sim_runtime.sh" "$@"
