#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

usage() {
  cat <<'EOF'
Usage:
  publish_walk_mode.sh <action> [--mode-id N]

Actions:
  start      -> 1000 + mode_id
  switch     -> 2000 + mode_id
  start_lc   -> 10
  stop       -> 11
  zero       -> 12
  estop      -> 13

Examples:
  ./publish_walk_mode.sh start --mode-id 2
  ./publish_walk_mode.sh switch --mode-id 1
  ./publish_walk_mode.sh stop
EOF
}

[[ $# -ge 1 ]] || { usage; exit 2; }
if [[ "$1" == "-h" || "$1" == "--help" ]]; then
  usage
  exit 0
fi

ACTION="$1"
shift
MODE_ID=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode-id)
      MODE_ID="${2:-}"
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

CONTROL_WORD=0
case "${ACTION}" in
  start)
    CONTROL_WORD=$((1000 + MODE_ID))
    ;;
  switch)
    CONTROL_WORD=$((2000 + MODE_ID))
    ;;
  start_lc)
    CONTROL_WORD=10
    ;;
  stop)
    CONTROL_WORD=11
    ;;
  zero)
    CONTROL_WORD=12
    ;;
  estop)
    CONTROL_WORD=13
    ;;
  *)
    usage
    die "Unknown action: ${ACTION}"
    ;;
esac

source_ros_workspace
log_info "Publishing /humanoid/rl/walk_mode = ${CONTROL_WORD}"
exec ros2 topic pub --once /humanoid/rl/walk_mode std_msgs/msg/Int32 "{data: ${CONTROL_WORD}}"
