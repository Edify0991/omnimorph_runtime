#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${WORKSPACE_DIR}"

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

BRIDGE_EXE="./install/rl_master/lib/rl_master/realsense_camera_bridge.py"
if [[ ! -x "${BRIDGE_EXE}" ]]; then
  echo "bridge executable not found: ${BRIDGE_EXE}" >&2
  echo "Build the workspace first: colcon build --packages-select rl_master" >&2
  exit 1
fi

CAMERA_NAME="${CAMERA_NAME:-camera}"
CAMERA_NAMESPACE="${CAMERA_NAMESPACE:-}"
CAMERA_ROOT="/${CAMERA_NAME}"
if [[ -n "${CAMERA_NAMESPACE}" ]]; then
  CAMERA_ROOT="/${CAMERA_NAMESPACE#/}/${CAMERA_NAME}"
fi

INPUT_COLOR_TOPIC="${INPUT_COLOR_TOPIC:-${CAMERA_ROOT}/color/image_raw}"
INPUT_DEPTH_TOPIC="${INPUT_DEPTH_TOPIC:-${CAMERA_ROOT}/depth/image_rect_raw}"
INPUT_COLOR_INFO_TOPIC="${INPUT_COLOR_INFO_TOPIC:-${CAMERA_ROOT}/color/camera_info}"
INPUT_DEPTH_INFO_TOPIC="${INPUT_DEPTH_INFO_TOPIC:-${CAMERA_ROOT}/depth/camera_info}"
OUTPUT_COLOR_TOPIC="${OUTPUT_COLOR_TOPIC:-/humanoid/camera/color/image_raw}"
OUTPUT_DEPTH_TOPIC="${OUTPUT_DEPTH_TOPIC:-/humanoid/camera/depth/image_raw}"
OUTPUT_COLOR_INFO_TOPIC="${OUTPUT_COLOR_INFO_TOPIC:-/humanoid/camera/color/camera_info}"
OUTPUT_DEPTH_INFO_TOPIC="${OUTPUT_DEPTH_INFO_TOPIC:-/humanoid/camera/depth/camera_info}"
OUTPUT_COLOR_COMPRESSED_TOPIC="${OUTPUT_COLOR_COMPRESSED_TOPIC:-/humanoid/camera/color/image_raw/compressed}"
OUTPUT_DEPTH_COMPRESSED_TOPIC="${OUTPUT_DEPTH_COMPRESSED_TOPIC:-/humanoid/camera/depth/image_raw/compressed}"
OUTPUT_FEATURE_TOPIC="${OUTPUT_FEATURE_TOPIC:-/humanoid/camera/features}"
TARGET_WIDTH="${TARGET_WIDTH:-424}"
TARGET_HEIGHT="${TARGET_HEIGHT:-240}"
OUTPUT_FPS="${OUTPUT_FPS:-15.0}"
DEPTH_SCALE="${DEPTH_SCALE:-0.001}"
JPEG_QUALITY="${JPEG_QUALITY:-80}"
COLOR_COMPRESSED_FORMAT="${COLOR_COMPRESSED_FORMAT:-jpeg}"
DEPTH_COMPRESSED_FORMAT="${DEPTH_COMPRESSED_FORMAT:-png}"

exec "${BRIDGE_EXE}" \
  --ros-args \
  -p input_color_topic:="${INPUT_COLOR_TOPIC}" \
  -p input_depth_topic:="${INPUT_DEPTH_TOPIC}" \
  -p input_color_info_topic:="${INPUT_COLOR_INFO_TOPIC}" \
  -p input_depth_info_topic:="${INPUT_DEPTH_INFO_TOPIC}" \
  -p output_color_topic:="${OUTPUT_COLOR_TOPIC}" \
  -p output_depth_topic:="${OUTPUT_DEPTH_TOPIC}" \
  -p output_color_info_topic:="${OUTPUT_COLOR_INFO_TOPIC}" \
  -p output_depth_info_topic:="${OUTPUT_DEPTH_INFO_TOPIC}" \
  -p output_color_compressed_topic:="${OUTPUT_COLOR_COMPRESSED_TOPIC}" \
  -p output_depth_compressed_topic:="${OUTPUT_DEPTH_COMPRESSED_TOPIC}" \
  -p output_feature_topic:="${OUTPUT_FEATURE_TOPIC}" \
  -p target_width:="${TARGET_WIDTH}" \
  -p target_height:="${TARGET_HEIGHT}" \
  -p output_fps:="${OUTPUT_FPS}" \
  -p depth_scale:="${DEPTH_SCALE}" \
  -p jpeg_quality:="${JPEG_QUALITY}" \
  -p color_compressed_format:="${COLOR_COMPRESSED_FORMAT}" \
  -p depth_compressed_format:="${DEPTH_COMPRESSED_FORMAT}" \
  "$@"
