cmake_minimum_required(VERSION 3.16)
project(joint_motor_test)

get_filename_component(RL_MASTER_ROOT_DIR_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../rl_master" ABSOLUTE)
add_compile_definitions(RL_MASTER_ROOT_DIR="${RL_MASTER_ROOT_DIR_PATH}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rl_master REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(joint_motor_test_runner
  src/main.cpp
  src/joint_motor_test_runner.cpp
  ../rl_master/dds_protocol.cpp
  ../rl_master/deploy_state_machine.cpp
)

target_include_directories(joint_motor_test_runner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/../rl_master/include
)

ament_target_dependencies(joint_motor_test_runner
  ament_index_cpp
  rclcpp
  std_msgs
  rl_master
)

target_link_libraries(joint_motor_test_runner
  yaml-cpp
)

install(TARGETS
  joint_motor_test_runner
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config docs launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
