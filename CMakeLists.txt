cmake_minimum_required(VERSION 3.16)
project(ev_ads_runtime LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "默认构建类型" FORCE)
endif()

option(EV_ADS_BUILD_ROS2_NATIVE "从根 CMake 直接编译 ROS2 C++ runtime 包；部署时推荐保持 OFF 并使用 colcon target" OFF)
option(EV_ADS_BUILD_TESTS "构建 Mac/本机 GoogleTest 项目级测试" ON)
option(EV_ADS_ENABLE_COLCON_TARGETS "提供 colcon 构建和 ros2 launch 的根目录快捷 target" ON)

set(EV_ADS_ROS_DISTRO "humble" CACHE STRING "ROS2 发行版")
set(EV_ADS_ROS_SETUP "/opt/ros/${EV_ADS_ROS_DISTRO}/setup.bash" CACHE FILEPATH "ROS2 setup.bash 路径")
set(EV_ADS_ROS_WORKSPACE "${CMAKE_CURRENT_SOURCE_DIR}/ros2_ws" CACHE PATH "ROS2 工作区路径")
set(EV_ADS_RUNTIME_PACKAGE_DIR "${EV_ADS_ROS_WORKSPACE}/src/ev_ads_runtime_cpp")
set(EV_ADS_COLCON_BUILD_TYPE "${CMAKE_BUILD_TYPE}" CACHE STRING "colcon 构建类型")

if(EV_ADS_BUILD_ROS2_NATIVE)
  find_package(ament_cmake QUIET)
  if(ament_cmake_FOUND)
    message(STATUS "已找到 ROS2/ament_cmake，根 CMake 将编译真实 runtime 包: ev_ads_runtime_cpp")
    add_subdirectory("${EV_ADS_RUNTIME_PACKAGE_DIR}" "${CMAKE_BINARY_DIR}/ev_ads_runtime_cpp")
  else()
    message(STATUS "未找到 ROS2/ament_cmake，跳过真实 runtime 的原生 CMake 编译；RK3588 上 source ROS 后可重新配置，或使用 ros2_workspace_build target 调用 colcon。")
  endif()
endif()

if(EV_ADS_ENABLE_COLCON_TARGETS AND UNIX)
  set(EV_ADS_COLCON_BUILD_SCRIPT "${CMAKE_CURRENT_BINARY_DIR}/ros2_workspace_build.sh")
  set(EV_ADS_RUN_FAKE_SCRIPT "${CMAKE_CURRENT_BINARY_DIR}/run_ev_ads_fake.sh")
  set(EV_ADS_RUN_HARDWARE_SCRIPT "${CMAKE_CURRENT_BINARY_DIR}/run_ev_ads_hardware.sh")

  file(GENERATE OUTPUT "${EV_ADS_COLCON_BUILD_SCRIPT}" CONTENT [=[
set -e
if [ ! -f "${EV_ADS_ROS_SETUP}" ]; then
  echo "未找到 ROS2 环境: ${EV_ADS_ROS_SETUP}" >&2
  exit 2
fi
if ! command -v colcon >/dev/null 2>&1; then
  echo "未找到 colcon，请先安装 python3-colcon-common-extensions" >&2
  exit 3
fi
source "${EV_ADS_ROS_SETUP}"
cd "${EV_ADS_ROS_WORKSPACE}"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE="${EV_ADS_COLCON_BUILD_TYPE}"
]=])

  file(GENERATE OUTPUT "${EV_ADS_RUN_FAKE_SCRIPT}" CONTENT [=[
set -e
if [ ! -f "${EV_ADS_ROS_SETUP}" ]; then
  echo "未找到 ROS2 环境: ${EV_ADS_ROS_SETUP}" >&2
  exit 2
fi
if [ ! -f "${EV_ADS_ROS_WORKSPACE}/install/setup.bash" ]; then
  echo "未找到已构建工作区: ${EV_ADS_ROS_WORKSPACE}/install/setup.bash，请先执行 cmake --build <build-dir> --target ros2_workspace_build" >&2
  exit 3
fi
source "${EV_ADS_ROS_SETUP}"
source "${EV_ADS_ROS_WORKSPACE}/install/setup.bash"
ros2 launch ev_ads_runtime_cpp ev_ads_runtime.launch.xml use_fakes:=true perception_mode:=scripted
]=])

  file(GENERATE OUTPUT "${EV_ADS_RUN_HARDWARE_SCRIPT}" CONTENT [=[
set -e
if [ ! -f "${EV_ADS_ROS_SETUP}" ]; then
  echo "未找到 ROS2 环境: ${EV_ADS_ROS_SETUP}" >&2
  exit 2
fi
if [ ! -f "${EV_ADS_ROS_WORKSPACE}/install/setup.bash" ]; then
  echo "未找到已构建工作区: ${EV_ADS_ROS_WORKSPACE}/install/setup.bash，请先执行 cmake --build <build-dir> --target ros2_workspace_build" >&2
  exit 3
fi
source "${EV_ADS_ROS_SETUP}"
source "${EV_ADS_ROS_WORKSPACE}/install/setup.bash"
ros2 launch ev_ads_runtime_cpp ev_ads_runtime.launch.xml use_fakes:=false perception_mode:=model
]=])

  add_custom_target(ros2_workspace_build
    COMMAND
      ${CMAKE_COMMAND} -E env
        "EV_ADS_ROS_SETUP=${EV_ADS_ROS_SETUP}"
        "EV_ADS_ROS_WORKSPACE=${EV_ADS_ROS_WORKSPACE}"
        "EV_ADS_COLCON_BUILD_TYPE=${EV_ADS_COLCON_BUILD_TYPE}"
        /bin/bash "${EV_ADS_COLCON_BUILD_SCRIPT}"
    USES_TERMINAL
    COMMENT "使用根目录 CMake target 调用 colcon 构建真实 ROS2 runtime"
  )

  add_custom_target(run_ev_ads_fake
    COMMAND
      ${CMAKE_COMMAND} -E env
        "EV_ADS_ROS_SETUP=${EV_ADS_ROS_SETUP}"
        "EV_ADS_ROS_WORKSPACE=${EV_ADS_ROS_WORKSPACE}"
        /bin/bash "${EV_ADS_RUN_FAKE_SCRIPT}"
    USES_TERMINAL
    COMMENT "从根目录启动 EV-ADS fake/scripted 演示链路"
  )

  add_custom_target(run_ev_ads_hardware
    COMMAND
      ${CMAKE_COMMAND} -E env
        "EV_ADS_ROS_SETUP=${EV_ADS_ROS_SETUP}"
        "EV_ADS_ROS_WORKSPACE=${EV_ADS_ROS_WORKSPACE}"
        /bin/bash "${EV_ADS_RUN_HARDWARE_SCRIPT}"
    USES_TERMINAL
    COMMENT "从根目录启动 EV-ADS 真实硬件/模型链路"
  )
endif()

if(EV_ADS_BUILD_TESTS)
  include(FetchContent)

  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  )
  FetchContent_MakeAvailable(googletest)

  enable_testing()
  include(GoogleTest)
  add_subdirectory(test)
endif()
