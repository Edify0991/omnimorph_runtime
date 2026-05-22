#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

usage() {
  cat <<'EOF'
Usage:
  reference_pose_replay_test.sh --params-file <bridge.yaml> [-- <extra ros args...>]

Purpose:
  Start mujoco_sim_bridge in "reference pose replay" test mode.
  In this mode, during RUNNING the backend stops using actuator ctrl/torque
  for policy execution and instead writes the current reference motion frame
  directly into MuJoCo qpos/qvel/base pose each control tick.

Examples:
  script/reference_pose_replay_test.sh \
    --params-file ${OMNIMORPH_RUNTIME_ROOT}/src/omnimorph_sim2sim/mujoco_sim2sim/config/jc01_legs_engineai_walk_sim2sim.yaml \
    -- -p startup_mode_id:=2 -p enable_viewer:=false

Notes:
  - Launch your Python/C++ viewer frontend in another terminal as usual.
  - This script only changes the backend bridge execution path.
EOF
}

PARAMS_FILE=""
NODE_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --params-file)
      PARAMS_FILE="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      NODE_ARGS=("$@")
      break
      ;;
    *)
      die "Unknown argument: $1"
      ;;
  esac
done

[[ -n "${PARAMS_FILE}" ]] || { usage; die "--params-file is required"; }
[[ -f "${PARAMS_FILE}" ]] || die "params file not found: ${PARAMS_FILE}"

print_banner "MuJoCo Reference Pose Replay Test"
source_ros_workspace

CMD=(
  ros2 run mujoco_sim2sim mujoco_sim_bridge
  --ros-args
  --params-file "${PARAMS_FILE}"
  -p enable_reference_pose_replay_test:=true
)
if [[ ${#NODE_ARGS[@]} -gt 0 ]]; then
  CMD+=("${NODE_ARGS[@]}")
fi

log_info "Starting backend in reference pose replay test mode ..."
log_info "${CMD[*]}"
exec "${CMD[@]}"
