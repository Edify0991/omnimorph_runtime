#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

usage() {
  cat <<'EOF'
Usage: dds_selfcheck.sh [options]

Default behavior is read-only checks (no control words published).

Options:
  --hz-seconds N          Seconds for topic hz probing (default: 3)
  --publish-smoke         Publish a safe smoke sequence to control topics:
                          teleop zero + walk_mode STOP_POLICY(11)
  --publish-sequence CSV  Publish explicit walk_mode sequence, e.g. "11,12,1000" or "1003"
  -h, --help              Show this help
EOF
}

HZ_SECONDS=3
PUBLISH_SMOKE=false
PUBLISH_SEQUENCE=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --hz-seconds)
      HZ_SECONDS="$2"
      shift 2
      ;;
    --publish-smoke)
      PUBLISH_SMOKE=true
      shift
      ;;
    --publish-sequence)
      PUBLISH_SEQUENCE="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "Unknown option: $1"
      ;;
  esac
done

if ! [[ "${HZ_SECONDS}" =~ ^[0-9]+$ ]] || [[ "${HZ_SECONDS}" -le 0 ]]; then
  die "--hz-seconds must be a positive integer"
fi

print_banner "DDS Deploy Self-Check"
source_ros_workspace

command -v ros2 >/dev/null 2>&1 || die "ros2 command not found in current environment"

run_with_timeout() {
  local sec="$1"
  shift
  if command -v timeout >/dev/null 2>&1; then
    timeout "${sec}" "$@"
  else
    "$@"
  fi
}

check_topic_type() {
  local topic="$1"
  local expected_type="$2"

  local actual_type
  actual_type="$(ros2 topic type "${topic}" 2>/dev/null || true)"
  if [[ -z "${actual_type}" ]]; then
    log_error "Missing topic: ${topic}"
    return 1
  fi

  if [[ "${actual_type}" != "${expected_type}" ]]; then
    log_error "Type mismatch: ${topic} expected=${expected_type}, actual=${actual_type}"
    return 1
  fi

  log_info "Topic OK: ${topic} (${actual_type})"
  return 0
}

check_topic_hz() {
  local topic="$1"
  local out
  out="$(run_with_timeout "${HZ_SECONDS}" ros2 topic hz "${topic}" 2>&1 || true)"

  if grep -q "average rate" <<<"${out}"; then
    log_info "Rate OK: ${topic}"
    echo "${out}" | tail -n 1
    return 0
  fi

  log_warn "Rate probe inconclusive: ${topic}"
  echo "${out}" | tail -n 3
  return 1
}

echo
log_info "[1/4] Checking required topic types"

FAIL_COUNT=0
check_topic_type "/imu/yesense" "sensor_msgs/msg/Imu" || FAIL_COUNT=$((FAIL_COUNT + 1))
check_topic_type "/humanoid/rl/state" "std_msgs/msg/Float32MultiArray" || FAIL_COUNT=$((FAIL_COUNT + 1))
check_topic_type "/humanoid/rl/command" "std_msgs/msg/Float32MultiArray" || FAIL_COUNT=$((FAIL_COUNT + 1))
check_topic_type "/humanoid/rl/teleop" "geometry_msgs/msg/Twist" || FAIL_COUNT=$((FAIL_COUNT + 1))
check_topic_type "/humanoid/rl/walk_mode" "std_msgs/msg/Int32" || FAIL_COUNT=$((FAIL_COUNT + 1))

echo
log_info "[2/4] Checking endpoint connectivity"
for topic in \
  "/imu/yesense" \
  "/humanoid/rl/state" \
  "/humanoid/rl/command" \
  "/humanoid/rl/teleop" \
  "/humanoid/rl/walk_mode"; do
  echo "--- ${topic} ---"
  ros2 topic info "${topic}" 2>&1 || true
done

echo
log_info "[3/4] Probing live rates"
check_topic_hz "/imu/yesense" || true
check_topic_hz "/humanoid/rl/state" || true

echo
log_info "[4/4] Optional publish checks"
if [[ "${PUBLISH_SMOKE}" == "true" ]]; then
  log_warn "Publish smoke enabled: teleop zero + walk_mode STOP_POLICY(11)"
  ros2 topic pub --once /humanoid/rl/teleop geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
  ros2 topic pub --once /humanoid/rl/walk_mode std_msgs/msg/Int32 "{data: 11}"
  log_info "Smoke publish done"
fi

if [[ -n "${PUBLISH_SEQUENCE}" ]]; then
  log_warn "Publishing explicit walk_mode sequence: ${PUBLISH_SEQUENCE}"
  IFS=',' read -r -a MODES <<< "${PUBLISH_SEQUENCE}"
  for mode in "${MODES[@]}"; do
    mode_trimmed="$(echo "${mode}" | xargs)"
    if ! [[ "${mode_trimmed}" =~ ^-?[0-9]+$ ]]; then
      log_warn "Skip invalid mode token: ${mode_trimmed}"
      continue
    fi
    ros2 topic pub --once /humanoid/rl/walk_mode std_msgs/msg/Int32 "{data: ${mode_trimmed}}"
    sleep 0.2
  done
  log_info "Sequence publish done"
fi

echo
if [[ "${FAIL_COUNT}" -eq 0 ]]; then
  log_info "Self-check completed: required topic types all matched"
else
  log_warn "Self-check completed with ${FAIL_COUNT} required topic/type failures"
  exit 2
fi
