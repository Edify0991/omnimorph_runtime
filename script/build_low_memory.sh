#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

packages=()
if [[ "$#" -gt 0 ]]; then
  packages=("$@")
fi

source_ros_setup() {
  local setup_file="$1"
  set +u
  # shellcheck disable=SC1090
  source "${setup_file}"
  set -u
}

if [[ -z "${ROS_DISTRO:-}" ]]; then
  for distro in humble jazzy iron galactic foxy; do
    if [[ -f "/opt/ros/${distro}/setup.bash" ]]; then
      source_ros_setup "/opt/ros/${distro}/setup.bash"
      break
    fi
  done
elif [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  source_ros_setup "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-1}"
export MAKEFLAGS="${MAKEFLAGS:--j1 -l1}"

cmake_args=(
  --no-warn-unused-cli
  -Wno-dev
  -DCMAKE_BUILD_TYPE=Release
  -DRL_MASTER_LOW_MEMORY_BUILD=ON
  -DMUJOCO_SIM2SIM_LOW_MEMORY_BUILD=ON
  -DRL_MASTER_ENABLE_NATIVE_TUNING=OFF
  -DMUJOCO_SIM2SIM_ENABLE_NATIVE_TUNING=OFF
)

if [[ "${OMNIMORPH_LOW_MEMORY_DISABLE_UNITREE_SDK2:-1}" == "1" ]]; then
  cmake_args+=(-DRL_MASTER_ENABLE_UNITREE_SDK2=OFF)
fi

if [[ "${OMNIMORPH_LOW_MEMORY_BUILD_TEST_TOOLS:-0}" != "1" ]]; then
  cmake_args+=(-DRL_MASTER_BUILD_TEST_TOOLS=OFF)
fi

colcon_args=(build --parallel-workers 1 --event-handlers console_direct+)
if [[ "${#packages[@]}" -gt 0 ]]; then
  colcon_args+=(--packages-select "${packages[@]}")
fi
colcon_args+=(--cmake-args "${cmake_args[@]}")

echo "[INFO] Low-memory build in ${WORKSPACE_DIR}"
echo "[INFO] CMAKE_BUILD_PARALLEL_LEVEL=${CMAKE_BUILD_PARALLEL_LEVEL}"
echo "[INFO] MAKEFLAGS=${MAKEFLAGS}"
echo "[INFO] colcon ${colcon_args[*]}"

cd "${WORKSPACE_DIR}"
exec colcon "${colcon_args[@]}"
