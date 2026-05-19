#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "Use: source script/dev_env.sh" >&2
  exit 1
fi

__jc01_saved_shell_opts="$(set +o)"
set +e +u
set +o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

eval "${__jc01_saved_shell_opts}"
unset __jc01_saved_shell_opts

restore_nounset=0
if [[ -o nounset ]]; then
  restore_nounset=1
  set +u
fi

source_ros_workspace

if [[ "${restore_nounset}" -eq 1 ]]; then
  set -u
fi

export JC01_DEPLOY_ROOT="${WORKSPACE_DIR}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-${WORKSPACE_DIR}/log/ros2}"
mkdir -p "${ROS_LOG_DIR}" >/dev/null 2>&1 || true

# Prefer system Python over any auto-activated conda Python. ROS Humble here is
# built against system Python, and mixing it with conda Python can cause import
# errors or shell exits after `source`.
system_bin_dirs=(
  "/usr/bin"
  "/bin"
  "/usr/sbin"
  "/sbin"
)

joined_system_bins="$(IFS=:; echo "${system_bin_dirs[*]}")"
if [[ -n "${PATH:-}" ]]; then
  export PATH="${joined_system_bins}:${PATH}"
else
  export PATH="${joined_system_bins}"
fi

# Prefer system/ROS runtime libraries ahead of conda-provided copies.
system_lib_dirs=(
  "/usr/lib/x86_64-linux-gnu"
  "/lib/x86_64-linux-gnu"
)
if [[ -n "${ROS_DISTRO:-}" ]]; then
  system_lib_dirs+=("/opt/ros/${ROS_DISTRO}/lib")
fi

joined_system_libs="$(IFS=:; echo "${system_lib_dirs[*]}")"
if [[ -n "${LD_LIBRARY_PATH:-}" ]]; then
  export LD_LIBRARY_PATH="${joined_system_libs}:${LD_LIBRARY_PATH}"
else
  export LD_LIBRARY_PATH="${joined_system_libs}"
fi

if [[ -n "${CONDA_PREFIX:-}" ]]; then
  log_warn "Detected active conda environment: ${CONDA_PREFIX}"
  log_warn "dev_env.sh now prioritizes system Python ahead of conda on PATH"
fi

log_info "Environment ready"
log_info "JC01_DEPLOY_ROOT=${JC01_DEPLOY_ROOT}"
log_info "ROS setup: ${ROS_DISTRO:-unknown}"
log_info "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
log_info "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
log_info "ROS_LOG_DIR=${ROS_LOG_DIR}"
log_info "python3=$(command -v python3)"
