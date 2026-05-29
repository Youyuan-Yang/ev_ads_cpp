cmake_minimum_required(VERSION 3.16)
project(ev_ads_interfaces)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(msg_files
  msg/MmWaveVital.msg
  msg/VehicleMotion.msg
  msg/FrontRisk.msg
  msg/BlindSpotState.msg
  msg/DriverState.msg
  msg/RiskState.msg
  msg/WarningCommand.msg
  msg/BrakeCommand.msg
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs builtin_interfaces
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
