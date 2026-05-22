cmake_minimum_required(VERSION 3.10)
project(imu_communication_yesense)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Threads REQUIRED)

# Create executable
add_executable(imu_communication_yesense
  src/main_imu.cpp
  src/yesense_imu_ros_node.cpp
  src/yesense_imu_decode/yesense_main.cpp
  src/yesense_imu_decode/analysis_data.cpp
)

# Include directories
target_include_directories(imu_communication_yesense PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /opt/kpl/include
)

# Link libraries
ament_target_dependencies(imu_communication_yesense
  rclcpp
  sensor_msgs
)

target_link_libraries(imu_communication_yesense
  Threads::Threads
)

# Install targets
install(TARGETS imu_communication_yesense
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

