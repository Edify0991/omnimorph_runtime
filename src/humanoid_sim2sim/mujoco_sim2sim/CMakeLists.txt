cmake_minimum_required(VERSION 3.16)
project(mujoco_sim2sim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rl_master REQUIRED)

find_path(MUJOCO_INCLUDE_DIR
  NAMES mujoco/mujoco.h
  HINTS
    $ENV{MUJOCO_ROOT}
    $ENV{MUJOCO_PATH}
  PATH_SUFFIXES include
)

find_library(MUJOCO_LIBRARY
  NAMES mujoco libmujoco
  HINTS
    $ENV{MUJOCO_ROOT}
    $ENV{MUJOCO_PATH}
  PATH_SUFFIXES lib bin
)

if(MUJOCO_INCLUDE_DIR AND MUJOCO_LIBRARY)
  add_executable(mujoco_sim_bridge
    src/main.cpp
    src/mujoco_sim_bridge.cpp
  )

  target_include_directories(mujoco_sim_bridge PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${MUJOCO_INCLUDE_DIR}
  )

  ament_target_dependencies(mujoco_sim_bridge
    rclcpp
    std_msgs
    rl_master
  )

  target_link_libraries(mujoco_sim_bridge
    ${MUJOCO_LIBRARY}
  )

  install(TARGETS mujoco_sim_bridge
    DESTINATION lib/${PROJECT_NAME}
  )

  message(STATUS "mujoco_sim2sim: MuJoCo found, executable will be built.")
else()
  message(WARNING
    "mujoco_sim2sim: MuJoCo not found. Set MUJOCO_ROOT (or MUJOCO_PATH) so CMake can find "
    "mujoco/mujoco.h and libmujoco. Package will be installed without executable.")
endif()

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config docs
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
