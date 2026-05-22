#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

usage() {
  cat <<'USAGE'
Usage:
  sim2real_engineai.sh [options]

Options:
  --mode-id <int>                 startup deploy mode_id (default: 0)
  --skip-precheck                 skip validate_deploy_config.py
  --precheck-skip-onnx            run precheck with --skip-onnx
  --precheck-python <path|name>   python used by precheck only (default: current python3)
  --auto-start-mode               publish START control word (1000 + mode_id) after solver starts
  --auto-start-delay <seconds>    delay before auto-start publish (default: 3.0)
  -h, --help                      show this help
USAGE
}

MODE_ID=0
SKIP_PRECHECK="false"
PRECHECK_SKIP_ONNX="false"
PRECHECK_PYTHON=""
AUTO_START_MODE="false"
AUTO_START_DELAY="3.0"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode-id)
      MODE_ID="${2:-}"
      shift 2
      ;;
    --skip-precheck)
      SKIP_PRECHECK="true"
      shift
      ;;
    --precheck-skip-onnx)
      PRECHECK_SKIP_ONNX="true"
      shift
      ;;
    --precheck-python)
      PRECHECK_PYTHON="${2:-}"
      shift 2
      ;;
    --auto-start-mode)
      AUTO_START_MODE="true"
      shift
      ;;
    --auto-start-delay)
      AUTO_START_DELAY="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "Unknown argument: $1"
      ;;
  esac
done

[[ "${MODE_ID}" =~ ^[0-9]+$ ]] || die "--mode-id must be a non-negative integer"
print_banner "Morph Runtime Sim2Real (Single-Process RL_solver)"

CURRENT_PYTHON="$(command -v python3 || true)"
[[ -n "${CURRENT_PYTHON}" ]] || die "python3 not found in PATH"

if [[ -z "${PRECHECK_PYTHON}" ]]; then
  if [[ "${CURRENT_PYTHON}" == *"/anaconda"* || "${CURRENT_PYTHON}" == *"/miniconda"* || "${CURRENT_PYTHON}" == *"/mambaforge"* ]] && [[ -x "/usr/bin/python3" ]]; then
    PRECHECK_PYTHON="/usr/bin/python3"
    log_warn "Detected conda python as default interpreter. Use /usr/bin/python3 for precheck by default."
  else
    PRECHECK_PYTHON="${CURRENT_PYTHON}"
  fi
elif [[ "${PRECHECK_PYTHON}" != */* ]]; then
  PRECHECK_PYTHON="$(command -v "${PRECHECK_PYTHON}" || true)"
fi
[[ -n "${PRECHECK_PYTHON}" && -x "${PRECHECK_PYTHON}" ]] || die "invalid --precheck-python: ${PRECHECK_PYTHON}"
log_info "Precheck python: ${PRECHECK_PYTHON}"

if [[ "${SKIP_PRECHECK}" != "true" ]]; then
  VALIDATOR="${WORKSPACE_DIR}/src/humanoid_rl_controller/rl_master/tools/analysis/validate_deploy_config.py"
  [[ -f "${VALIDATOR}" ]] || die "validator not found: ${VALIDATOR}"
  CHECK_CMD=("${PRECHECK_PYTHON}" "${VALIDATOR}" --mode-id "${MODE_ID}")
  if [[ "${PRECHECK_SKIP_ONNX}" == "true" ]]; then
    CHECK_CMD+=(--skip-onnx)
  fi
  log_info "Running deploy precheck for mode_id=${MODE_ID} ..."
  "${CHECK_CMD[@]}"
fi

source_ros_workspace
export ROS_LOG_DIR="${ROS_LOG_DIR:-${WORKSPACE_DIR}/log/ros2}"
mkdir -p "${ROS_LOG_DIR}" >/dev/null 2>&1 || true

if [[ "${AUTO_START_MODE}" == "true" ]]; then
  (
    sleep "${AUTO_START_DELAY}"
    ros2 topic pub --once /humanoid/rl/mode_control std_msgs/msg/Int32 \
      "{data: $((1000 + MODE_ID))}" >/dev/null 2>&1 || true
  ) &
  log_info "Scheduled START control word: $((1000 + MODE_ID)) after ${AUTO_START_DELAY}s"
fi

CMD=("${SCRIPT_DIR}/start_rl_solver.sh" --mode-id "${MODE_ID}")
log_info "Launching fused sim2real runtime ..."
log_info "${CMD[*]}"
exec "${CMD[@]}"
