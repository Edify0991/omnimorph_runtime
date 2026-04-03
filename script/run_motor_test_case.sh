#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ $# -lt 1 ]]; then
  echo "Usage: run_motor_test_case.sh <executable> [args...]"
  exit 2
fi

TARGET="$1"
shift

exec "${SCRIPT_DIR}/run_ros_executable.sh" motor_test "${TARGET}" --build-if-missing -- "$@"

