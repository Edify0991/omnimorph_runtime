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
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rcl_interfaces REQUIRED)
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
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../install/SharedMemory/include
)

# Link directories
target_link_directories(imu_communication_yesense PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../install/SharedMemory/lib
)

# Link libraries
ament_target_dependencies(imu_communication_yesense
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  std_srvs
  action_msgs
  rclcpp_action
  rcl_interfaces
)

target_link_libraries(imu_communication_yesense
  SharedMemory
  Threads::Threads
  /opt/ros/humble/lib/librosidl_typesupport_cpp.so
  /opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_cpp.so
  /opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_introspection_cpp.so
  /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_cpp.so
  /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_introspection_cpp.so
  /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_cpp.so
  /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_introspection_cpp.so
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

