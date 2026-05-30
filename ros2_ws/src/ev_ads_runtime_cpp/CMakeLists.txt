cmake_minimum_required(VERSION 3.16)
project(ev_ads_runtime_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OpenCV REQUIRED)
find_package(SQLite3 REQUIRED)

if(TARGET SQLite3::SQLite3)
  set(SQLITE_TARGET SQLite3::SQLite3)
else()
  set(SQLITE_TARGET SQLite::SQLite3)
endif()

# 自定义消息已经合并到本包，避免 ros2_ws/src 下出现 interfaces/bringup/runtime 三包分散。
set(MSG_FILES
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
  ${MSG_FILES}
  DEPENDENCIES std_msgs builtin_interfaces
  ADD_LINTER_TESTS
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

include_directories(include ${OpenCV_INCLUDE_DIRS})

set(COMMON_DEPS
  rclcpp
  std_msgs
  sensor_msgs
)

# OpenCV ONNX 推理封装：后置靠近识别和驾驶员 DMS 共用。
add_library(onnx_yolo_detector src/onnx_yolo_detector.cpp)
target_link_libraries(onnx_yolo_detector ${OpenCV_LIBS})

# 事件存储封装：默认 SQLite/WAL，保留 JSONL 兼容后端。
add_library(event_store src/event_store.cpp)
target_link_libraries(event_store ${SQLITE_TARGET})

function(configure_runtime_node target)
  ament_target_dependencies(${target} ${COMMON_DEPS})
  target_link_libraries(${target} ${cpp_typesupport_target})
endfunction()

# 采集层节点。
add_executable(camera_capture_node src/camera_capture_node.cpp)
configure_runtime_node(camera_capture_node)
target_link_libraries(camera_capture_node ${OpenCV_LIBS})

add_executable(imu_motion_node src/imu_motion_node.cpp)
configure_runtime_node(imu_motion_node)

add_executable(mmwave_vital_node src/mmwave_vital_node.cpp)
configure_runtime_node(mmwave_vital_node)

# 感知层节点。
add_executable(front_risk_node src/front_risk_node.cpp)
configure_runtime_node(front_risk_node)

add_executable(rear_blind_spot_node src/rear_blind_spot_node.cpp)
configure_runtime_node(rear_blind_spot_node)
target_link_libraries(rear_blind_spot_node onnx_yolo_detector ${OpenCV_LIBS})

add_executable(driver_attention_node src/driver_attention_node.cpp)
configure_runtime_node(driver_attention_node)
target_link_libraries(driver_attention_node onnx_yolo_detector ${OpenCV_LIBS})

# 决策、提示和记录节点。
add_executable(risk_fusion_node src/risk_fusion_node.cpp)
configure_runtime_node(risk_fusion_node)

add_executable(terminal_hmi_node src/terminal_hmi_node.cpp)
configure_runtime_node(terminal_hmi_node)

add_executable(event_recorder_node src/event_recorder_node.cpp)
configure_runtime_node(event_recorder_node)
target_link_libraries(event_recorder_node event_store ${SQLITE_TARGET})

install(TARGETS
  onnx_yolo_detector
  event_store
  camera_capture_node
  imu_motion_node
  mmwave_vital_node
  front_risk_node
  rear_blind_spot_node
  driver_attention_node
  risk_fusion_node
  terminal_hmi_node
  event_recorder_node
  DESTINATION lib/${PROJECT_NAME}
)

# 根目录 config 是唯一配置源；安装时放到 ROS2 包 share 下，保持 ros2 launch 可用。
install(FILES
  ../../../config/ev_ads_runtime.launch.xml
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY ../../../config/scenarios
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  ../../../models/onnx/rear_yolo.onnx
  ../../../models/onnx/driver_face_yunet.onnx
  ../../../models/onnx/driver_dms_yolo.onnx
  DESTINATION share/${PROJECT_NAME}/models/onnx
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
