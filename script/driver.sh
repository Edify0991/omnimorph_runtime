#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

print_banner "JC Driver bringup"

if ! command -v expect >/dev/null 2>&1; then
  die "expect is required by driver.sh (sudo apt install expect)"
fi

CANDIDATES=()
if [[ -n "${MOTOR_TEST_EXEC:-}" ]]; then
  CANDIDATES+=("${MOTOR_TEST_EXEC}")
fi

CANDIDATES+=(
  "${WORKSPACE_DIR}/src/jc-driver/build/tests/motor_test/motor_test"
  "${WORKSPACE_DIR}/src/jc-driver/build/jc01/bin/motor_test"
  "/home/nvidia/Documents/jc_driver/build/jc01/bin/motor_test"
)

EXEC_PATH=""
for path in "${CANDIDATES[@]}"; do
  if [[ -x "${path}" ]]; then
    EXEC_PATH="${path}"
    break
  fi
done

[[ -n "${EXEC_PATH}" ]] || die "motor_test executable not found"

APP_START_CMD="${APP_START_CMD:-AppStart POS}"
ENABLE_CMD="${ENABLE_CMD:-AppM18Enable EN_DRV}"
START_DELAY_SEC="${START_DELAY_SEC:-2}"
ENABLE_DELAY_SEC="${ENABLE_DELAY_SEC:-10}"

export EXEC_PATH APP_START_CMD ENABLE_CMD START_DELAY_SEC ENABLE_DELAY_SEC
log_info "Launching motor_test and sending init commands"

/usr/bin/expect <<'EOF'
set timeout -1
set exec_path $env(EXEC_PATH)
set app_start_cmd $env(APP_START_CMD)
set enable_cmd $env(ENABLE_CMD)
set start_delay $env(START_DELAY_SEC)
set enable_delay $env(ENABLE_DELAY_SEC)

spawn $exec_path
expect {
  -re {\[Command\] >} {
    send "$app_start_cmd\r"
  }
}

after [expr {int($start_delay * 1000)}]
expect {
  -re {\[Command\] >} {
    send "$enable_cmd\r"
  }
}

after [expr {int($enable_delay * 1000)}]
puts "driver init complete, keep process attached (Ctrl+C to exit)"
interact
EOF

