#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

usage() {
  cat <<'EOF'
Usage:
  check_ros_domain.sh [--domain ID] [--strict] [--timeout SEC]

Checks whether the selected ROS_DOMAIN_ID can already see ROS 2 nodes.

Options:
  --domain ID     Domain to check; otherwise use existing ROS_DOMAIN_ID.
  --strict        Exit non-zero if any node is visible in this domain.
  --timeout SEC   ros2 node list timeout in seconds (default: 3).
  -h, --help      Show this help.
EOF
}

STRICT=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --domain)
      export ROS_DOMAIN_ID="${2:-}"
      shift 2
      ;;
    --strict)
      STRICT=true
      shift
      ;;
    --timeout)
      export OMNIMORPH_ROS_DOMAIN_SCAN_TIMEOUT_SEC="${2:-}"
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

print_banner "ROS Domain Preflight"
source_ros_workspace

if [[ "${STRICT}" == "true" ]]; then
  export OMNIMORPH_ROS_DOMAIN_SCAN=strict
else
  export OMNIMORPH_ROS_DOMAIN_SCAN=warn
fi

prepare_ros_network_env "rmw_fastrtps_cpp" || exit 1
