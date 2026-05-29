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
find_package(ev_ads_interfaces REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})

set(COMMON_DEPS
  rclcpp
  std_msgs
  sensor_msgs
  ev_ads_interfaces
)

add_library(yolo_onnx src/yolo_onnx.cpp)
target_link_libraries(yolo_onnx ${OpenCV_LIBS})

add_executable(front_node_cpp src/front_node_cpp.cpp)
ament_target_dependencies(front_node_cpp ${COMMON_DEPS})

add_executable(rear_node_cpp src/rear_node_cpp.cpp)
ament_target_dependencies(rear_node_cpp ${COMMON_DEPS})
target_link_libraries(rear_node_cpp yolo_onnx ${OpenCV_LIBS})

add_executable(driver_monitor_node_cpp src/driver_monitor_node_cpp.cpp)
ament_target_dependencies(driver_monitor_node_cpp ${COMMON_DEPS})
target_link_libraries(driver_monitor_node_cpp yolo_onnx ${OpenCV_LIBS})

add_executable(fusion_node_cpp src/fusion_node_cpp.cpp)
ament_target_dependencies(fusion_node_cpp ${COMMON_DEPS})

add_executable(imu_node_cpp src/imu_node_cpp.cpp)
ament_target_dependencies(imu_node_cpp ${COMMON_DEPS})

add_executable(camera_node_cpp src/camera_node_cpp.cpp)
ament_target_dependencies(camera_node_cpp ${COMMON_DEPS})
target_link_libraries(camera_node_cpp ${OpenCV_LIBS})

add_executable(mmwave_node_cpp src/mmwave_node_cpp.cpp)
ament_target_dependencies(mmwave_node_cpp ${COMMON_DEPS})

add_executable(hmi_node_cpp src/hmi_node_cpp.cpp)
ament_target_dependencies(hmi_node_cpp ${COMMON_DEPS})

add_executable(event_logger_node_cpp src/event_logger_node_cpp.cpp)
ament_target_dependencies(event_logger_node_cpp ${COMMON_DEPS})

install(TARGETS
  yolo_onnx
  front_node_cpp
  rear_node_cpp
  driver_monitor_node_cpp
  fusion_node_cpp
  imu_node_cpp
  camera_node_cpp
  mmwave_node_cpp
  hmi_node_cpp
  event_logger_node_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config scenarios
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
