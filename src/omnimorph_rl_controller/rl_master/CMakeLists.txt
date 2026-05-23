cmake_minimum_required(VERSION 3.16)
project(rl_master)

add_definitions(-DRL_MASTER_ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(RL_MASTER_ENABLE_ARCH_TUNING "Enable architecture-aware compile tuning." ON)
option(RL_MASTER_ENABLE_NATIVE_TUNING "Use -march=native for local machine tuning." OFF)
set(RL_MASTER_ARM_BASELINE "armv8-a" CACHE STRING "Baseline ARM architecture when native tuning is off.")

string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" RL_MASTER_SYSTEM_PROCESSOR)
set(RL_MASTER_ARCH_KIND "generic")
if(RL_MASTER_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(RL_MASTER_ARCH_KIND "aarch64")
elseif(RL_MASTER_SYSTEM_PROCESSOR MATCHES "x86_64|amd64|x64")
  set(RL_MASTER_ARCH_KIND "x86_64")
endif()

message(STATUS "rl_master: detected architecture='${RL_MASTER_ARCH_KIND}' (CMAKE_SYSTEM_PROCESSOR='${CMAKE_SYSTEM_PROCESSOR}')")

add_library(rl_master_arch_flags INTERFACE)
if(RL_MASTER_ENABLE_ARCH_TUNING AND (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang"))
  if(RL_MASTER_ENABLE_NATIVE_TUNING)
    target_compile_options(rl_master_arch_flags INTERFACE
      $<$<COMPILE_LANGUAGE:CXX>:-march=native>
      $<$<COMPILE_LANGUAGE:CXX>:-mtune=native>
      $<$<COMPILE_LANGUAGE:CXX>:-fno-omit-frame-pointer>
    )
    message(STATUS "rl_master: native tuning enabled (-march=native -mtune=native).")
  elseif(RL_MASTER_ARCH_KIND STREQUAL "aarch64")
    target_compile_options(rl_master_arch_flags INTERFACE
      $<$<COMPILE_LANGUAGE:CXX>:-march=${RL_MASTER_ARM_BASELINE}>
      $<$<COMPILE_LANGUAGE:CXX>:-fno-omit-frame-pointer>
    )
    message(STATUS "rl_master: ARM baseline tuning enabled (-march=${RL_MASTER_ARM_BASELINE}).")
  elseif(RL_MASTER_ARCH_KIND STREQUAL "x86_64")
    target_compile_options(rl_master_arch_flags INTERFACE
      $<$<COMPILE_LANGUAGE:CXX>:-fno-omit-frame-pointer>
    )
    message(STATUS "rl_master: x86_64 generic tuning enabled (portable baseline).")
  endif()
endif()

if(RL_MASTER_ARCH_KIND STREQUAL "aarch64")
  target_compile_definitions(rl_master_arch_flags INTERFACE RL_MASTER_ARCH_AARCH64=1)
elseif(RL_MASTER_ARCH_KIND STREQUAL "x86_64")
  target_compile_definitions(rl_master_arch_flags INTERFACE RL_MASTER_ARCH_X86_64=1)
else()
  target_compile_definitions(rl_master_arch_flags INTERFACE RL_MASTER_ARCH_GENERIC=1)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(SharedMemory REQUIRED)
find_package(unitree_hg QUIET)
find_package(yaml-cpp REQUIRED)
find_package(SymEngine CONFIG QUIET)

set(_rl_master_ros_prefix_hints "")
if(DEFINED ENV{ROS_DISTRO} AND NOT "$ENV{ROS_DISTRO}" STREQUAL "")
  list(APPEND _rl_master_ros_prefix_hints
    "/opt/ros/$ENV{ROS_DISTRO}"
    "/opt/ros/$ENV{ROS_DISTRO}/lib/${CMAKE_LIBRARY_ARCHITECTURE}/cmake/pinocchio"
  )
endif()
list(APPEND _rl_master_ros_prefix_hints
  "/opt/ros/humble"
  "/opt/ros/humble/lib/${CMAKE_LIBRARY_ARCHITECTURE}/cmake/pinocchio"
  "/opt/ros/foxy"
  "/opt/ros/foxy/lib/${CMAKE_LIBRARY_ARCHITECTURE}/cmake/pinocchio"
)
list(REMOVE_DUPLICATES _rl_master_ros_prefix_hints)

find_package(pinocchio CONFIG REQUIRED
  HINTS ${_rl_master_ros_prefix_hints}
)
find_package(Threads REQUIRED)
find_library(ZSTD_LIBRARY NAMES zstd REQUIRED)
find_library(LZ4_LIBRARY NAMES lz4 REQUIRED)

set(ONNXRUNTIME_ROOT "" CACHE PATH "ONNX Runtime installation prefix.")
if(NOT ONNXRUNTIME_ROOT AND DEFINED ENV{ONNXRUNTIME_ROOT})
  set(ONNXRUNTIME_ROOT "$ENV{ONNXRUNTIME_ROOT}")
endif()
if(NOT ONNXRUNTIME_ROOT AND DEFINED ENV{ONNXRUNTIME_DIR})
  set(ONNXRUNTIME_ROOT "$ENV{ONNXRUNTIME_DIR}")
endif()

find_path(ONNXRUNTIME_INCLUDE_DIR
  NAMES onnxruntime_cxx_api.h
  HINTS
    ${ONNXRUNTIME_ROOT}
    /usr/local
    /usr
  PATH_SUFFIXES
    include
    include/onnxruntime
    include/onnxruntime/core/session
)

find_library(ONNXRUNTIME_LIBRARY
  NAMES onnxruntime libonnxruntime
  HINTS
    ${ONNXRUNTIME_ROOT}
    /usr/local
    /usr
  PATH_SUFFIXES
    lib
    lib64
)

if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIBRARY)
  message(FATAL_ERROR
    "rl_master: ONNX Runtime not found.\n"
    "Set ONNXRUNTIME_ROOT (or env ONNXRUNTIME_ROOT/ONNXRUNTIME_DIR) to a prefix that contains:\n"
    "  include/onnxruntime_cxx_api.h (or include/onnxruntime/...)\n"
    "  lib/libonnxruntime.so")
endif()

if(NOT TARGET onnxruntime::onnxruntime)
  add_library(onnxruntime::onnxruntime UNKNOWN IMPORTED)
  set_target_properties(onnxruntime::onnxruntime PROPERTIES
    IMPORTED_LOCATION "${ONNXRUNTIME_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${ONNXRUNTIME_INCLUDE_DIR}"
  )
endif()

message(STATUS "rl_master: ONNX Runtime include='${ONNXRUNTIME_INCLUDE_DIR}'")
message(STATUS "rl_master: ONNX Runtime library='${ONNXRUNTIME_LIBRARY}'")

set(RL_MASTER_SYMENGINE_TARGET "")
if(TARGET symengine::symengine)
  set(RL_MASTER_SYMENGINE_TARGET symengine::symengine)
elseif(TARGET SymEngine::symengine)
  set(RL_MASTER_SYMENGINE_TARGET SymEngine::symengine)
endif()
if(RL_MASTER_SYMENGINE_TARGET)
  message(STATUS "rl_master: SymEngine target='${RL_MASTER_SYMENGINE_TARGET}'")
else()
  message(STATUS "rl_master: SymEngine not found; continuing without it.")
endif()

set(_rl_master_runtime_rpath_dirs "$ORIGIN")
if(ONNXRUNTIME_LIBRARY)
  get_filename_component(_onnxruntime_lib_dir "${ONNXRUNTIME_LIBRARY}" DIRECTORY)
  if(_onnxruntime_lib_dir)
    list(APPEND _rl_master_runtime_rpath_dirs "${_onnxruntime_lib_dir}")
  endif()
endif()
if(ONNXRUNTIME_ROOT)
  list(APPEND _rl_master_runtime_rpath_dirs "${ONNXRUNTIME_ROOT}/lib" "${ONNXRUNTIME_ROOT}/lib64")
endif()
if(DEFINED ENV{CONDA_ENGINEAI_PREFIX})
  list(APPEND _rl_master_runtime_rpath_dirs "$ENV{CONDA_ENGINEAI_PREFIX}/lib")
endif()
if(DEFINED ENV{ROS_DISTRO} AND NOT "$ENV{ROS_DISTRO}" STREQUAL "")
  list(APPEND _rl_master_runtime_rpath_dirs "/opt/ros/$ENV{ROS_DISTRO}/lib")
  if(CMAKE_LIBRARY_ARCHITECTURE)
    list(APPEND _rl_master_runtime_rpath_dirs "/opt/ros/$ENV{ROS_DISTRO}/lib/${CMAKE_LIBRARY_ARCHITECTURE}")
  endif()
endif()
list(APPEND _rl_master_runtime_rpath_dirs "/opt/ros/humble/lib" "/opt/ros/foxy/lib")
if(CMAKE_LIBRARY_ARCHITECTURE)
  list(APPEND _rl_master_runtime_rpath_dirs
    "/opt/ros/humble/lib/${CMAKE_LIBRARY_ARCHITECTURE}"
    "/opt/ros/foxy/lib/${CMAKE_LIBRARY_ARCHITECTURE}")
endif()
list(REMOVE_DUPLICATES _rl_master_runtime_rpath_dirs)
set(RL_MASTER_RPATH "")
if(_rl_master_runtime_rpath_dirs)
  string(JOIN ":" RL_MASTER_RPATH ${_rl_master_runtime_rpath_dirs})
endif()

add_library(rl_master_runtime SHARED
  dds_protocol.cpp
  deploy_state_machine.cpp
  external_observation_provider.cpp
  filters/base_velocity_kalman_filter.cpp
  logging/mcap_writer.cpp
  logging/runtime_recorder.cpp
  mode_profile_registry.cpp
  RL_controller.cpp
  onnx_policy_runner.cpp
  observation_builder.cpp
  policy_runtime.cpp
  pinocchio_motion_features.cpp
  reference_motion_provider.cpp
  runtime/integrated_controller_runtime.cpp
  runtime/realtime_utils.cpp
  robot_state.cpp
  math_tool.cpp
)

target_include_directories(rl_master_runtime PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(rl_master_runtime PUBLIC
  RL_MASTER_ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)
target_include_directories(rl_master_runtime PRIVATE
  ${ONNXRUNTIME_INCLUDE_DIR}
)
ament_target_dependencies(rl_master_runtime PUBLIC
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
)
target_link_libraries(rl_master_runtime PUBLIC
  pinocchio::pinocchio
)
target_link_libraries(rl_master_runtime PRIVATE
  onnxruntime::onnxruntime
  Threads::Threads
  yaml-cpp
  ${ZSTD_LIBRARY}
  ${LZ4_LIBRARY}
)
target_link_libraries(rl_master_runtime PRIVATE
  rl_master_arch_flags
)
target_compile_features(rl_master_runtime PUBLIC cxx_std_17)

if(RL_MASTER_RPATH)
  set_target_properties(rl_master_runtime PROPERTIES
    BUILD_RPATH "${RL_MASTER_RPATH}"
    INSTALL_RPATH "${RL_MASTER_RPATH}"
  )
endif()

add_library(rl_master_jc01_kinematics STATIC
  kinematics/jc01/ankle_kinematics.cpp
  kinematics/jc01/knee_kinematics.cpp
  kinematics/jc01/kin_conv.cpp
)
target_include_directories(rl_master_jc01_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /usr/include/eigen3
)
target_link_libraries(rl_master_jc01_kinematics PRIVATE rl_master_arch_flags)

add_library(rl_master_robot_kinematics STATIC
  kinematics/robot_kinematics_adapter.cpp
)
target_include_directories(rl_master_robot_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /usr/include/eigen3
)
target_link_libraries(rl_master_robot_kinematics
  PUBLIC
    rl_master_jc01_kinematics
  PRIVATE
    rl_master_arch_flags
)

add_executable(test_kine
  test_kine.cpp
)
target_include_directories(test_kine PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  /usr/include/eigen3
)
target_link_libraries(test_kine
  rl_master_jc01_kinematics
  rl_master_arch_flags
)

add_executable(RL_solver
  filters/moving_average_filter.cpp
  rl_solver.cpp
  solver/motor_io_backend.cpp
  solver/motor_shm_io.cpp
  solver/robot_solver.cpp
  solver_dds_bridge.cpp
)
target_include_directories(RL_solver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /usr/include/eigen3
)
target_include_directories(RL_solver PRIVATE
  ${ONNXRUNTIME_INCLUDE_DIR}
)
ament_target_dependencies(RL_solver
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  SharedMemory
)
if(unitree_hg_FOUND)
  target_sources(RL_solver PRIVATE
    solver/unitree_g1_motor_io.cpp
  )
  ament_target_dependencies(RL_solver
    unitree_hg
  )
  target_compile_definitions(RL_solver PRIVATE RL_MASTER_HAS_UNITREE_HG=1)
  set(_unitree_ros2_root_hints "")
  set(_unitree_ros2_common_include_hints "")
  set(_unitree_ros2_common_source_hints "")
  if(DEFINED ENV{UNITREE_ROS2_ROOT} AND NOT "$ENV{UNITREE_ROS2_ROOT}" STREQUAL "")
    list(APPEND _unitree_ros2_root_hints "$ENV{UNITREE_ROS2_ROOT}")
  endif()
  if(EXISTS "/home/unitree/unitree_ros2")
    list(APPEND _unitree_ros2_root_hints "/home/unitree/unitree_ros2")
  endif()
  list(REMOVE_DUPLICATES _unitree_ros2_root_hints)
  foreach(_unitree_root IN LISTS _unitree_ros2_root_hints)
    list(APPEND _unitree_ros2_common_include_hints
      "${_unitree_root}/example/src/include"
      "${_unitree_root}/example/src"
      "${_unitree_root}/include"
    )
    list(APPEND _unitree_ros2_common_source_hints
      "${_unitree_root}/example/src/src/common"
      "${_unitree_root}/src/common"
    )
  endforeach()
  find_path(UNITREE_ROS2_COMMON_INCLUDE_DIR
    NAMES common/motor_crc_hg.h
    HINTS
      ${_unitree_ros2_common_include_hints}
      /opt/unitree_ros2/include
      /usr/local/include
      /usr/include
  )
  find_file(UNITREE_ROS2_MOTOR_CRC_HG_SOURCE
    NAMES motor_crc_hg.cpp
    HINTS
      ${_unitree_ros2_common_source_hints}
  )
  if(UNITREE_ROS2_COMMON_INCLUDE_DIR)
    target_include_directories(RL_solver PRIVATE
      ${UNITREE_ROS2_COMMON_INCLUDE_DIR}
      ${UNITREE_ROS2_COMMON_INCLUDE_DIR}/common
    )
    if(UNITREE_ROS2_MOTOR_CRC_HG_SOURCE)
      target_sources(RL_solver PRIVATE ${UNITREE_ROS2_MOTOR_CRC_HG_SOURCE})
      target_compile_definitions(RL_solver PRIVATE OMNIMORPH_HAS_UNITREE_CRC=1)
      message(STATUS
        "rl_master: Unitree G1 DDS motor IO enabled with CRC header at ${UNITREE_ROS2_COMMON_INCLUDE_DIR} "
        "and source ${UNITREE_ROS2_MOTOR_CRC_HG_SOURCE}")
    else()
      message(WARNING
        "rl_master: common/motor_crc_hg.h was found at ${UNITREE_ROS2_COMMON_INCLUDE_DIR}, "
        "but motor_crc_hg.cpp was not. CRC will remain disabled until the official source file is available.")
    endif()
  else()
    message(WARNING
      "rl_master: unitree_hg found, but common/motor_crc_hg.h was not. "
      "Unitree G1 DDS motor IO will build without filling the official LowCmd CRC field. "
      "Set UNITREE_ROS2_ROOT to the official unitree_ros2 checkout to enable CRC.")
  endif()
else()
  message(STATUS "rl_master: unitree_hg not found; Unitree G1 DDS motor IO backend disabled.")
endif()
target_link_libraries(RL_solver
  rl_master_runtime
  onnxruntime::onnxruntime
  Threads::Threads
  SharedMemory::SharedMemory
  yaml-cpp
  rl_master_robot_kinematics
  rl_master_arch_flags
)
if(RL_MASTER_SYMENGINE_TARGET)
  target_link_libraries(RL_solver PRIVATE ${RL_MASTER_SYMENGINE_TARGET})
endif()
target_compile_features(RL_solver PRIVATE cxx_std_17)

if(RL_MASTER_RPATH)
  set_target_properties(RL_solver PROPERTIES
    BUILD_RPATH "${RL_MASTER_RPATH}"
    INSTALL_RPATH "${RL_MASTER_RPATH}"
  )
endif()

install(TARGETS
  rl_master_arch_flags
  rl_master_runtime
  EXPORT export_rl_master
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  RL_solver
  test_kine
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config docs
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/omnimorph_ops_gui.py
  scripts/realsense_camera_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY scripts/omnimorph_ops_console
  DESTINATION lib/${PROJECT_NAME}
)

install(EXPORT export_rl_master
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs SharedMemory pinocchio)
ament_export_targets(export_rl_master HAS_LIBRARY_TARGET)

ament_package()
