#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

cd "${WORKSPACE_DIR}"
source_ros_workspace

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

COLOR_PROFILE="${COLOR_PROFILE:-424x240x15}"
DEPTH_PROFILE="${DEPTH_PROFILE:-424x240x15}"
CAMERA_NAME="${CAMERA_NAME:-camera}"
CAMERA_NAMESPACE="${CAMERA_NAMESPACE:-}"

if ! ros2 pkg prefix realsense2_camera >/dev/null 2>&1; then
  echo "realsense2_camera package not found in current ROS environment." >&2
  echo "Install or source the RealSense ROS driver first." >&2
  exit 1
fi

exec ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:="${CAMERA_NAMESPACE}" \
  camera_name:="${CAMERA_NAME}" \
  enable_color:=true \
  enable_depth:=true \
  enable_infra1:=false \
  enable_infra2:=false \
  pointcloud.enable:=false \
  align_depth.enable:=false \
  rgb_camera.profile:="${COLOR_PROFILE}" \
  depth_module.profile:="${DEPTH_PROFILE}" \
  "$@"
