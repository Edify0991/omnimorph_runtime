cmake_minimum_required(VERSION 3.16)
project(rl_master)

add_definitions(-DRL_MASTER_ROOT_DIR="${CMAKE_SOURCE_DIR}")

# ---------- 通用编译选项 ----------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ----------  find ROS 2 ----------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(SharedMemory REQUIRED)
find_package(yaml-cpp REQUIRED)

# ----------  find 外部库 ----------
# 指定 conda 环境前缀以便 cmake 绝对查找 onnxruntime 和 symengine
# 指向你打包好的 engineai 前缀（include 与 lib 在该目录下）
# 添加/usr/local/include到包含目录，这样就能找到onnxruntime_cxx_api.h文件
include_directories(/usr/local/include)

# 确保使用系统中安装的x86版本的symengine库，而不是engineai目录中的ARM版本
link_directories(/usr/local/lib)

# 手动指定onnxruntime的库文件
set(ONNXRUNTIME_LIB /usr/local/lib/libonnxruntime.so)

# 使用 SymEngine 的 CONFIG 包（并依赖于上面设置的 symengine_DIR）
find_package(SymEngine CONFIG REQUIRED)
find_package(Threads REQUIRED)

# ---------- RL_controller ----------
add_executable(RL_controller
  dds_protocol.cpp
  dds_robot_io.cpp
  deploy_state_machine.cpp
  external_observation_provider.cpp
  logging/structured_logger.cpp
  RL_controller.cpp
  onnx_policy_runner.cpp
  observation_builder.cpp
  reference_motion_provider.cpp
  runtime/realtime_utils.cpp
  robot_state.cpp
  sim2real_rl_controller.cpp
  math_tool.cpp
)

target_include_directories(RL_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(RL_controller PRIVATE
  ${CONDA_ENGINEAI_PREFIX}/include
  /usr/local/include
)
ament_target_dependencies(RL_controller rclcpp std_msgs sensor_msgs geometry_msgs)

target_link_libraries(RL_controller
  ${ONNXRUNTIME_LIB}
  Threads::Threads
  yaml-cpp
)
target_compile_features(RL_controller PRIVATE cxx_std_17)

set_target_properties(RL_controller PROPERTIES
  BUILD_RPATH "${CONDA_ENGINEAI_PREFIX}/lib"
  INSTALL_RPATH "${CONDA_ENGINEAI_PREFIX}/lib"
)

#-----------------------------test_kine-------------------------#
# 增加可执行文件
add_executable(test_kine 
                test_kine.cpp 
                Ankle_Kinematics.cpp  # 必须加入，因为 Ankle_Kinematics.h 的实现在这里
                Knee_Kinematics.cpp  # 必须加入，因为 Ankle_Kinematics.h 的实现在这里
              )

# 包含头文件路径
target_include_directories(test_kine PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    /usr/include/eigen3
)

#-----------------------------KinConv-------------------------#
add_library(KinConv  
    KinConv.cpp 
    Ankle_Kinematics.cpp
    Knee_Kinematics.cpp
    math_tool.cpp
)
target_include_directories(KinConv PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /usr/include/eigen3
)
target_link_directories(KinConv PRIVATE
  /usr/local/lib   # symengine
)

# ---------- RL_solver ----------
add_executable(RL_solver
  dds_protocol.cpp
  filters/moving_average_filter.cpp
  logging/structured_logger.cpp
  rl_solver.cpp
  runtime/realtime_utils.cpp
  solver/motor_shm_io.cpp
  solver/robot_solver.cpp
  solver_dds_bridge.cpp
  Knee_Kinematics.cpp
  Ankle_Kinematics.cpp
  math_tool.cpp
  KinConv.cpp
)
target_include_directories(RL_solver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../install/SharedMemory/include
  /usr/include/eigen3   # Eigen3 系统头文件，保留无妨
  /usr/local/include/symengine
)
ament_target_dependencies(RL_solver rclcpp std_msgs sensor_msgs SharedMemory)
target_link_libraries(RL_solver
  symengine
  Threads::Threads
  SharedMemory::SharedMemory
  yaml-cpp
)
target_compile_features(RL_solver PRIVATE cxx_std_17)

# ---------- 安装 ----------
install(TARGETS 
  RL_controller 
  RL_solver 
  test_kine
  KinConv
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY config docs
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)

ament_package()
