cmake_minimum_required(VERSION 3.16)
project(mujoco_sim2sim)
option(MUJOCO_SIM2SIM_ENABLE_VIEWER "Enable optional MuJoCo GLFW viewer support" ON)
option(MUJOCO_SIM2SIM_ENABLE_ARCH_TUNING "Enable architecture-aware compile tuning." ON)
option(MUJOCO_SIM2SIM_ENABLE_NATIVE_TUNING "Use -march=native for local machine tuning." OFF)
set(MUJOCO_SIM2SIM_ARM_BASELINE "armv8-a" CACHE STRING "Baseline ARM architecture when native tuning is off.")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" MUJOCO_SIM2SIM_SYSTEM_PROCESSOR)
set(MUJOCO_SIM2SIM_ARCH_KIND "generic")
if(MUJOCO_SIM2SIM_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(MUJOCO_SIM2SIM_ARCH_KIND "aarch64")
elseif(MUJOCO_SIM2SIM_SYSTEM_PROCESSOR MATCHES "x86_64|amd64|x64")
  set(MUJOCO_SIM2SIM_ARCH_KIND "x86_64")
endif()
message(STATUS "mujoco_sim2sim: detected architecture='${MUJOCO_SIM2SIM_ARCH_KIND}' (CMAKE_SYSTEM_PROCESSOR='${CMAKE_SYSTEM_PROCESSOR}')")

add_library(mujoco_sim2sim_arch_flags INTERFACE)
if(MUJOCO_SIM2SIM_ENABLE_ARCH_TUNING AND (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang"))
  if(MUJOCO_SIM2SIM_ENABLE_NATIVE_TUNING)
    target_compile_options(mujoco_sim2sim_arch_flags INTERFACE
      $<$<COMPILE_LANGUAGE:CXX>:-march=native>
      $<$<COMPILE_LANGUAGE:CXX>:-mtune=native>
      $<$<COMPILE_LANGUAGE:CXX>:-fno-omit-frame-pointer>
    )
    message(STATUS "mujoco_sim2sim: native tuning enabled (-march=native -mtune=native).")
  elseif(MUJOCO_SIM2SIM_ARCH_KIND STREQUAL "aarch64")
    target_compile_options(mujoco_sim2sim_arch_flags INTERFACE
      $<$<COMPILE_LANGUAGE:CXX>:-march=${MUJOCO_SIM2SIM_ARM_BASELINE}>
      $<$<COMPILE_LANGUAGE:CXX>:-fno-omit-frame-pointer>
    )
    message(STATUS "mujoco_sim2sim: ARM baseline tuning enabled (-march=${MUJOCO_SIM2SIM_ARM_BASELINE}).")
  elseif(MUJOCO_SIM2SIM_ARCH_KIND STREQUAL "x86_64")
    target_compile_options(mujoco_sim2sim_arch_flags INTERFACE
      $<$<COMPILE_LANGUAGE:CXX>:-fno-omit-frame-pointer>
    )
    message(STATUS "mujoco_sim2sim: x86_64 generic tuning enabled (portable baseline).")
  endif()
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
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
  set(MUJOCO_SIM2SIM_WITH_VIEWER OFF)
  set(MUJOCO_SIM2SIM_GLFW_LIB "")

  if(MUJOCO_SIM2SIM_ENABLE_VIEWER)
    find_package(glfw3 QUIET)
    if(TARGET glfw)
      set(MUJOCO_SIM2SIM_WITH_VIEWER ON)
      set(MUJOCO_SIM2SIM_GLFW_LIB glfw)
    elseif(TARGET glfw3)
      set(MUJOCO_SIM2SIM_WITH_VIEWER ON)
      set(MUJOCO_SIM2SIM_GLFW_LIB glfw3)
    else()
      find_library(MUJOCO_SIM2SIM_GLFW_RAW_LIB NAMES glfw glfw3)
      if(MUJOCO_SIM2SIM_GLFW_RAW_LIB)
        set(MUJOCO_SIM2SIM_WITH_VIEWER ON)
        set(MUJOCO_SIM2SIM_GLFW_LIB ${MUJOCO_SIM2SIM_GLFW_RAW_LIB})
      endif()
    endif()
  endif()

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
    geometry_msgs
    rl_master
  )

  target_link_libraries(mujoco_sim_bridge
    rl_master::rl_master_runtime
    ${MUJOCO_LIBRARY}
    mujoco_sim2sim_arch_flags
  )

  set(_mujoco_sim2sim_rpath_dirs "")
  if(DEFINED ONNXRUNTIME_LIBRARY AND ONNXRUNTIME_LIBRARY)
    get_filename_component(_mujoco_sim2sim_onnxruntime_lib_dir "${ONNXRUNTIME_LIBRARY}" DIRECTORY)
    if(_mujoco_sim2sim_onnxruntime_lib_dir)
      list(APPEND _mujoco_sim2sim_rpath_dirs "${_mujoco_sim2sim_onnxruntime_lib_dir}")
    endif()
  endif()
  list(REMOVE_DUPLICATES _mujoco_sim2sim_rpath_dirs)
  if(_mujoco_sim2sim_rpath_dirs)
    string(JOIN ":" MUJOCO_SIM2SIM_RPATH ${_mujoco_sim2sim_rpath_dirs})
    set_target_properties(mujoco_sim_bridge PROPERTIES
      BUILD_RPATH "${MUJOCO_SIM2SIM_RPATH}"
      INSTALL_RPATH "${MUJOCO_SIM2SIM_RPATH}"
      INSTALL_RPATH_USE_LINK_PATH TRUE
    )
  else()
    set_target_properties(mujoco_sim_bridge PROPERTIES
      INSTALL_RPATH_USE_LINK_PATH TRUE
    )
  endif()

  if(MUJOCO_SIM2SIM_WITH_VIEWER)
    target_compile_definitions(mujoco_sim_bridge PRIVATE MUJOCO_SIM2SIM_WITH_VIEWER=1)
    target_link_libraries(mujoco_sim_bridge ${MUJOCO_SIM2SIM_GLFW_LIB})
    find_package(OpenGL QUIET)
    if(OpenGL_FOUND)
      target_link_libraries(mujoco_sim_bridge OpenGL::GL)
    endif()
    message(STATUS "mujoco_sim2sim: GLFW viewer support enabled.")
  else()
    message(STATUS "mujoco_sim2sim: GLFW viewer support disabled.")
  endif()

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

install(PROGRAMS
  scripts/mujoco_sim_interactive_backend.py
  scripts/mujoco_sim_viewer_frontend.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
