cmake_minimum_required(VERSION 3.16)
project(unitree_g1_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(SharedMemory REQUIRED)
find_package(rl_master REQUIRED)
find_package(unitree_hg QUIET)

if(unitree_hg_FOUND)
  add_executable(unitree_g1_bridge
    src/unitree_g1_bridge.cpp
  )

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
    target_include_directories(unitree_g1_bridge PRIVATE
      ${UNITREE_ROS2_COMMON_INCLUDE_DIR}
      ${UNITREE_ROS2_COMMON_INCLUDE_DIR}/common
    )
    if(UNITREE_ROS2_MOTOR_CRC_HG_SOURCE)
      target_sources(unitree_g1_bridge PRIVATE ${UNITREE_ROS2_MOTOR_CRC_HG_SOURCE})
      target_compile_definitions(unitree_g1_bridge PRIVATE OMNIMORPH_HAS_UNITREE_CRC=1)
      message(STATUS
        "unitree_g1_bridge: using Unitree CRC header from ${UNITREE_ROS2_COMMON_INCLUDE_DIR} "
        "and source ${UNITREE_ROS2_MOTOR_CRC_HG_SOURCE}")
    else()
      message(WARNING
        "unitree_g1_bridge: common/motor_crc_hg.h was found at ${UNITREE_ROS2_COMMON_INCLUDE_DIR}, "
        "but motor_crc_hg.cpp was not. CRC will remain disabled until the official source file is available.")
    endif()
  else()
    message(WARNING
      "unitree_g1_bridge: unitree_hg was found, but common/motor_crc_hg.h was not. "
      "The bridge will build without filling the official LowCmd CRC field. "
      "Set UNITREE_ROS2_ROOT to the official unitree_ros2 checkout to enable CRC.")
  endif()

  ament_target_dependencies(unitree_g1_bridge
    rclcpp
    SharedMemory
    rl_master
    unitree_hg
  )
  target_link_libraries(unitree_g1_bridge
    SharedMemory::SharedMemory
  )

  install(TARGETS unitree_g1_bridge
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(STATUS
    "unitree_g1_bridge: unitree_hg package not found. "
    "Skipping legacy standalone bridge executable. "
    "Normal G1 deployment uses rl_master's in-process unitree_g1_dds backend; "
    "source the official Unitree ROS2 workspace only if you need this standalone diagnostic bridge.")
endif()

install(DIRECTORY docs
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
