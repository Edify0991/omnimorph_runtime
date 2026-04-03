#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

print_banner "JC Driver motor_test"

CANDIDATES=()
if [[ -n "${MOTOR_TEST_EXEC:-}" ]]; then
  CANDIDATES+=("${MOTOR_TEST_EXEC}")
fi

CANDIDATES+=(
  "${WORKSPACE_DIR}/src/jc-driver/build/tests/motor_test/motor_test"
  "${WORKSPACE_DIR}/src/jc-driver/build/jc01/bin/motor_test"
  "/home/nvidia/Documents/jc_driver/build/jc01/bin/motor_test"
)

EXECUTABLE_PATH=""
for path in "${CANDIDATES[@]}"; do
  if [[ -x "${path}" ]]; then
    EXECUTABLE_PATH="${path}"
    break
  fi
done

if [[ -z "${EXECUTABLE_PATH}" ]]; then
  die "motor_test executable not found. Build jc-driver tests first."
fi

log_info "Using executable: ${EXECUTABLE_PATH}"
exec "${EXECUTABLE_PATH}" "$@"

