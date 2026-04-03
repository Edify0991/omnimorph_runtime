#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() {
  cat <<'EOF'
Usage: motor_test_suite.sh [smoke|zero|all]

Modes:
  smoke   Run receive_test with finite iterations (safe default)
  zero    Run move_zero with configurable interpolation
  all     Run smoke then zero

Environment overrides:
  RECEIVE_ITERATIONS   (default: 1000)
  RECEIVE_RATE_HZ      (default: 500)
  RECEIVE_PRINT_EVERY  (default: 100)
  ZERO_DURATION_SEC    (default: 5.0)
  ZERO_RATE_HZ         (default: 1000)
  ZERO_PRINT_EVERY     (default: 400)
EOF
}

mode="${1:-smoke}"
case "${mode}" in
  -h|--help)
    usage
    exit 0
    ;;
  smoke|zero|all)
    ;;
  *)
    echo "Unknown mode: ${mode}"
    usage
    exit 2
    ;;
esac

run_smoke() {
  local iterations="${RECEIVE_ITERATIONS:-1000}"
  local rate_hz="${RECEIVE_RATE_HZ:-500}"
  local print_every="${RECEIVE_PRINT_EVERY:-100}"

  echo "[suite] run receive_test: iterations=${iterations}, rate_hz=${rate_hz}, print_every=${print_every}"
  "${SCRIPT_DIR}/run_motor_test_case.sh" receive_test \
    --iterations "${iterations}" \
    --rate-hz "${rate_hz}" \
    --print-every "${print_every}"
}

run_zero() {
  local duration_sec="${ZERO_DURATION_SEC:-5.0}"
  local rate_hz="${ZERO_RATE_HZ:-1000}"
  local print_every="${ZERO_PRINT_EVERY:-400}"

  echo "[suite] run move_zero: duration_sec=${duration_sec}, rate_hz=${rate_hz}, print_every=${print_every}"
  "${SCRIPT_DIR}/run_motor_test_case.sh" move_zero \
    --duration-sec "${duration_sec}" \
    --rate-hz "${rate_hz}" \
    --print-every "${print_every}" \
    --no-wait
}

case "${mode}" in
  smoke)
    run_smoke
    ;;
  zero)
    run_zero
    ;;
  all)
    run_smoke
    run_zero
    ;;
esac
