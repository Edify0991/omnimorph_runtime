#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

usage() {
  cat <<'USAGE'
Usage:
  sim2sim_engineai_python.sh --model-path <robot.xml> [options]

Recommended Python GUI path:
  C++ fused backend (physics + controller) -> ROS2 frame stream -> Python MuJoCo viewer frontend

Options:
  --model-path <path>             MuJoCo XML/MJB model path (required)
  --mode-id <int>                 startup deploy mode_id for precheck/optional auto-start (default: 0)
  --control-hz <float>            control_hz launch arg (default: 100.0)
  --fixed-base <true|false>       fixed_base launch arg (default: false)
  --enable-viewer <bool>          enable_viewer launch arg (default: true)
  --show-left-ui <bool>           python viewer left panel (default: true)
  --show-right-ui <bool>          python viewer right panel (default: true)
  --pause-when-no-command <bool>  pause_when_no_command launch arg (default: false)
  --no-command-behavior <value>   hold_position|hold_last|zero_torque (default: hold_position)
  --actuator-control-mode <value> auto|torque|position (default: auto)
  --bridge-config <path>          override bridge yaml path
  --skip-precheck                 skip validate_deploy_config.py
  --precheck-skip-onnx            run precheck with --skip-onnx
  --precheck-python <path|name>   python used by precheck only (default: current python3)
  --auto-start-mode               publish START control word (1000 + mode_id) after launch starts
  --auto-start-delay <seconds>    delay before auto-start publish (default: 3.0)
  -h, --help                      show this help
USAGE
}

MODEL_PATH=""
MODE_ID=0
CONTROL_HZ="100.0"
FIXED_BASE="false"
ENABLE_VIEWER="true"
SHOW_LEFT_UI="true"
SHOW_RIGHT_UI="true"
PAUSE_WHEN_NO_COMMAND="false"
NO_COMMAND_BEHAVIOR="hold_position"
ACTUATOR_CONTROL_MODE="auto"
BRIDGE_CONFIG=""
SKIP_PRECHECK="false"
PRECHECK_SKIP_ONNX="false"
PRECHECK_PYTHON=""
AUTO_START_MODE="false"
AUTO_START_DELAY="3.0"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --model-path)
      MODEL_PATH="${2:-}"
      shift 2
      ;;
    --mode-id)
      MODE_ID="${2:-}"
      shift 2
      ;;
    --control-hz)
      CONTROL_HZ="${2:-}"
      shift 2
      ;;
    --fixed-base)
      FIXED_BASE="${2:-}"
      shift 2
      ;;
    --enable-viewer)
      ENABLE_VIEWER="${2:-}"
      shift 2
      ;;
    --show-left-ui)
      SHOW_LEFT_UI="${2:-}"
      shift 2
      ;;
    --show-right-ui)
      SHOW_RIGHT_UI="${2:-}"
      shift 2
      ;;
    --pause-when-no-command)
      PAUSE_WHEN_NO_COMMAND="${2:-}"
      shift 2
      ;;
    --no-command-behavior)
      NO_COMMAND_BEHAVIOR="${2:-}"
      shift 2
      ;;
    --actuator-control-mode)
      ACTUATOR_CONTROL_MODE="${2:-}"
      shift 2
      ;;
    --bridge-config)
      BRIDGE_CONFIG="${2:-}"
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

[[ -n "${MODEL_PATH}" ]] || { usage; die "--model-path is required"; }
[[ "${MODE_ID}" =~ ^[0-9]+$ ]] || die "--mode-id must be a non-negative integer"
[[ -f "${MODEL_PATH}" ]] || log_warn "model file not found yet: ${MODEL_PATH} (launch may fail if path is wrong)"

print_banner "Morph Runtime Sim2Sim (Python GUI Frontend)"
log_info "This path keeps the friendly Python MuJoCo GUI, while the fused C++ backend owns both physics and policy/control loop."

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
  RESOLVED_PRECHECK_PYTHON="$(command -v "${PRECHECK_PYTHON}" || true)"
  if [[ "${PRECHECK_PYTHON}" == "python3" ]] && [[ "${RESOLVED_PRECHECK_PYTHON}" == *"/anaconda"* || "${RESOLVED_PRECHECK_PYTHON}" == *"/miniconda"* || "${RESOLVED_PRECHECK_PYTHON}" == *"/mambaforge"* ]] && [[ -x "/usr/bin/python3" ]]; then
    PRECHECK_PYTHON="/usr/bin/python3"
    log_warn "Requested precheck python3 resolves to conda: ${RESOLVED_PRECHECK_PYTHON}. Fallback to /usr/bin/python3."
  else
    PRECHECK_PYTHON="${RESOLVED_PRECHECK_PYTHON}"
  fi
fi
[[ -n "${PRECHECK_PYTHON}" && -x "${PRECHECK_PYTHON}" ]] || die "invalid --precheck-python: ${PRECHECK_PYTHON}"
log_info "Precheck python: ${PRECHECK_PYTHON}"

if [[ "${SKIP_PRECHECK}" != "true" ]]; then
  VALIDATOR="${WORKSPACE_DIR}/src/omnimorph_rl_controller/rl_master/tools/analysis/validate_deploy_config.py"
  [[ -f "${VALIDATOR}" ]] || die "validator not found: ${VALIDATOR}"
  CHECK_CMD=("${PRECHECK_PYTHON}" "${VALIDATOR}" --mode-id "${MODE_ID}")
  if [[ "${PRECHECK_SKIP_ONNX}" == "true" ]]; then
    CHECK_CMD+=(--skip-onnx)
  fi
  log_info "Running deploy precheck for mode_id=${MODE_ID} ..."
  "${CHECK_CMD[@]}"
fi

if [[ "${CURRENT_PYTHON}" == *"/anaconda"* || "${CURRENT_PYTHON}" == *"/miniconda"* || "${CURRENT_PYTHON}" == *"/mambaforge"* ]]; then
  log_warn "Detected conda python: ${CURRENT_PYTHON}. Switching to system python for ROS launch."
  export PATH="/usr/bin:/bin:${PATH}"
  hash -r
fi

source_ros_workspace
prepare_ros_network_env "rmw_fastrtps_cpp" || exit 1
export ROS_LOG_DIR="${ROS_LOG_DIR:-${WORKSPACE_DIR}/log/ros2}"
mkdir -p "${ROS_LOG_DIR}" >/dev/null 2>&1 || true

if [[ "${AUTO_START_MODE}" == "true" ]]; then
  (
    sleep "${AUTO_START_DELAY}"
    ros2 topic pub --once /omnimorph/rl/mode_control std_msgs/msg/Int32 \
      "{data: $((1000 + MODE_ID))}" >/dev/null 2>&1 || true
  ) &
  log_info "Scheduled START control word: $((1000 + MODE_ID)) after ${AUTO_START_DELAY}s"
fi

LAUNCH_CMD=(
  ros2 launch mujoco_sim2sim sim2sim_mujoco.launch.py
  "model_path:=${MODEL_PATH}"
  "backend:=python_frontend"
  "mode_id:=${MODE_ID}"
  "control_hz:=${CONTROL_HZ}"
  "fixed_base:=${FIXED_BASE}"
  "enable_viewer:=${ENABLE_VIEWER}"
  "show_left_ui:=${SHOW_LEFT_UI}"
  "show_right_ui:=${SHOW_RIGHT_UI}"
  "pause_when_no_command:=${PAUSE_WHEN_NO_COMMAND}"
  "no_command_behavior:=${NO_COMMAND_BEHAVIOR}"
  "actuator_control_mode:=${ACTUATOR_CONTROL_MODE}"
)
if [[ -n "${BRIDGE_CONFIG}" ]]; then
  LAUNCH_CMD+=("bridge_config:=${BRIDGE_CONFIG}")
fi

log_info "Launching python_frontend sim2sim path ..."
log_info "${LAUNCH_CMD[*]}"
exec "${LAUNCH_CMD[@]}"
