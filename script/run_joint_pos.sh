#!/usr/bin/env bash
set -euo pipefail

# run_joint_pos.sh
# Run the joint_pos executable from this workspace.
# Usage: ./run_joint_pos.sh [--use-install] [--input INPUT_FILE] [--output OUTPUT_FILE] [--extra-args "..."]
# Examples:
#   ./script/run_joint_pos.sh --use-install --input data.txt --output motor_action.csv
#   ./script/run_joint_pos.sh --extra-args "--help"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="${SCRIPT_DIR}/.."
LOG_DIR="${WS_ROOT}/log/run_joint_pos"
mkdir -p "${LOG_DIR}"

USE_INSTALL=false
INPUT_FILE=""
OUTPUT_FILE=""
EXTRA_ARGS=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --use-install)
      USE_INSTALL=true; shift;;
    --input)
      INPUT_FILE="$2"; shift 2;;
    --output)
      OUTPUT_FILE="$2"; shift 2;;
    --extra-args)
      EXTRA_ARGS="$2"; shift 2;;
    -h|--help)
      sed -n '1,200p' "${BASH_SOURCE[0]}"; exit 0;;
    *)
      echo "Unknown arg: $1"; exit 2;;
  esac
done

# Optionally source workspace install setup
if [ "${USE_INSTALL}" = true ] && [ -f "${WS_ROOT}/install/setup.sh" ]; then
  # shellcheck disable=SC1091
  source "${WS_ROOT}/install/setup.sh"
fi

# Possible locations for the built binary
CANDIDATES=(
  "${WS_ROOT}/install/humantic_robot/lib/humantic_robot/joint_pos"
  "${WS_ROOT}/install/humantic_robot/lib/joint_pos"
  "${WS_ROOT}/install/lib/humantic_robot/joint_pos"
  "${WS_ROOT}/build/humantic_robot/joint_pos"
  "${WS_ROOT}/build/humantic_robot/joint_pos"
)

JOINT_BIN=""
for p in "${CANDIDATES[@]}"; do
  if [ -x "$p" ]; then
    JOINT_BIN="$p"
    break
  fi
done

# If not found in candidates, try a find
if [ -z "${JOINT_BIN}" ]; then
  echo "joint_pos not found in common locations, searching..."
  JOINT_BIN=$(find "${WS_ROOT}" -maxdepth 4 -type f -name joint_pos -executable 2>/dev/null | head -n 1 || true)
fi

if [ -z "${JOINT_BIN}" ]; then
  echo "Error: joint_pos executable not found. Please build the workspace first (colcon build)."
  exit 1
fi

TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
STDOUT_LOG="${LOG_DIR}/joint_pos_stdout_${TIMESTAMP}.log"
STDERR_LOG="${LOG_DIR}/joint_pos_stderr_${TIMESTAMP}.log"

CMD=("${JOINT_BIN}")
if [ -n "${INPUT_FILE}" ]; then
  CMD+=("${INPUT_FILE}")
fi
if [ -n "${OUTPUT_FILE}" ]; then
  CMD+=("${OUTPUT_FILE}")
fi
if [ -n "${EXTRA_ARGS}" ]; then
  # shellcheck disable=SC2206
  ARGS_ARRAY=(${EXTRA_ARGS})
  CMD+=("${ARGS_ARRAY[@]}")
fi

echo "Running: ${CMD[*]}"
"${CMD[@]}" >"${STDOUT_LOG}" 2>"${STDERR_LOG}" || RUN_EXIT=$?
RUN_EXIT=${RUN_EXIT:-0}

if [ ${RUN_EXIT} -eq 0 ]; then
  echo "joint_pos finished successfully. Logs:"
  echo "  stdout: ${STDOUT_LOG}"
  echo "  stderr: ${STDERR_LOG}"
else
  echo "joint_pos exited with code ${RUN_EXIT}. Logs:"
  echo "  stdout: ${STDOUT_LOG}"
  echo "  stderr: ${STDERR_LOG}"
fi

exit ${RUN_EXIT}
