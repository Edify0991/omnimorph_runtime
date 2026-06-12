#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

usage() {
  cat <<'USAGE'
Usage:
  sim2sim_runtime.sh --model-path <robot.xml> [options]

Options:
  --model-path <path>             MuJoCo XML/MJB model path (required)
  --mode-id <int>                 startup deploy mode_id (default: 0)
  --control-hz <float>            control_hz launch arg (default: 500.0)
  --fixed-base <true|false>       fixed_base launch arg (default: false)
  --enable-fixed-base-zeroing <true|false>
                                  lock base in air for startup/required re-zeroing (default: true)
  --enable-fixed-base-hold-after-zeroing <true|false>
                                  keep base locked in hold after zeroing (default: true)
  --enable-release-before-running <true|false>
                                  release base before entering running (default: true)
  --post-release-settle-ticks <int>
                                  hold ticks after release before running (default: 200)
  --enable-prepose-snap <true|false>
                                  snap controlled joints to prepose_joint_q before zeroing (default: false)
  --enable-viewer <bool>          enable_viewer launch arg (default: true)
  --pause-when-no-command <bool>  pause stepping when controller output is inactive (default: false)
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
CONTROL_HZ="500.0"
FIXED_BASE="false"
ENABLE_FIXED_BASE_ZEROING="true"
ENABLE_FIXED_BASE_HOLD_AFTER_ZEROING="true"
ENABLE_RELEASE_BEFORE_RUNNING="true"
POST_RELEASE_SETTLE_TICKS="200"
ENABLE_PREPOSE_SNAP="false"
ENABLE_VIEWER="true"
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
    --enable-fixed-base-zeroing)
      ENABLE_FIXED_BASE_ZEROING="${2:-}"
      shift 2
      ;;
    --enable-fixed-base-hold-after-zeroing)
      ENABLE_FIXED_BASE_HOLD_AFTER_ZEROING="${2:-}"
      shift 2
      ;;
    --enable-release-before-running)
      ENABLE_RELEASE_BEFORE_RUNNING="${2:-}"
      shift 2
      ;;
    --post-release-settle-ticks)
      POST_RELEASE_SETTLE_TICKS="${2:-}"
      shift 2
      ;;
    --enable-prepose-snap)
      ENABLE_PREPOSE_SNAP="${2:-}"
      shift 2
      ;;
    --enable-viewer)
      ENABLE_VIEWER="${2:-}"
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

print_banner "Morph Runtime Sim2Sim (Single-Process Fused Runtime)"

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
  VALIDATOR="${WORKSPACE_DIR}/src/omnimorph_rl_controller/rl_master/tools/analysis/validate_deploy_config.py"
  [[ -f "${VALIDATOR}" ]] || die "validator not found: ${VALIDATOR}"
  CHECK_CMD=("${PRECHECK_PYTHON}" "${VALIDATOR}" --mode-id "${MODE_ID}")
  if [[ "${PRECHECK_SKIP_ONNX}" == "true" ]]; then
    CHECK_CMD+=(--skip-onnx)
  fi
  log_info "Running deploy precheck for mode_id=${MODE_ID} ..."
  "${CHECK_CMD[@]}"
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
  "backend:=cpp"
  "mode_id:=${MODE_ID}"
  "control_hz:=${CONTROL_HZ}"
  "fixed_base:=${FIXED_BASE}"
  "enable_fixed_base_zeroing:=${ENABLE_FIXED_BASE_ZEROING}"
  "enable_fixed_base_hold_after_zeroing:=${ENABLE_FIXED_BASE_HOLD_AFTER_ZEROING}"
  "enable_release_before_running:=${ENABLE_RELEASE_BEFORE_RUNNING}"
  "post_release_settle_ticks:=${POST_RELEASE_SETTLE_TICKS}"
  "enable_prepose_snap:=${ENABLE_PREPOSE_SNAP}"
  "enable_viewer:=${ENABLE_VIEWER}"
  "pause_when_no_command:=${PAUSE_WHEN_NO_COMMAND}"
  "no_command_behavior:=${NO_COMMAND_BEHAVIOR}"
  "actuator_control_mode:=${ACTUATOR_CONTROL_MODE}"
)
if [[ -n "${BRIDGE_CONFIG}" ]]; then
  LAUNCH_CMD+=("bridge_config:=${BRIDGE_CONFIG}")
fi

log_info "Launching fused sim2sim runtime ..."
log_info "${LAUNCH_CMD[*]}"
exec "${LAUNCH_CMD[@]}"
