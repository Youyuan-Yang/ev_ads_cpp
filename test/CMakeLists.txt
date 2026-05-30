cmake_minimum_required(VERSION 3.16)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(ev_ads_mac_tests LANGUAGES CXX)
  enable_testing()
  include(FetchContent)
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  )
  FetchContent_MakeAvailable(googletest)
  include(GoogleTest)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(SQLite3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

if(TARGET SQLite3::SQLite3)
  set(SQLITE_TARGET SQLite3::SQLite3)
else()
  set(SQLITE_TARGET SQLite::SQLite3)
endif()

set(RUNTIME_DIR "${CMAKE_CURRENT_LIST_DIR}/../ros2_ws/src/ev_ads_runtime_cpp")

add_library(ev_ads_core_for_test
  "${RUNTIME_DIR}/src/event_store.cpp"
)
target_include_directories(ev_ads_core_for_test PUBLIC
  "${RUNTIME_DIR}/include"
)
target_link_libraries(ev_ads_core_for_test PUBLIC ${SQLITE_TARGET})

add_library(ev_ads_yolo_for_test
  "${RUNTIME_DIR}/src/onnx_yolo_detector.cpp"
)
target_include_directories(ev_ads_yolo_for_test PUBLIC
  "${RUNTIME_DIR}/include"
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(ev_ads_yolo_for_test PUBLIC ${OpenCV_LIBS})

add_executable(test_risk_math_and_fusion test_risk_math_and_fusion.cpp)
target_link_libraries(test_risk_math_and_fusion PRIVATE ev_ads_core_for_test GTest::gtest_main)
gtest_discover_tests(test_risk_math_and_fusion)

add_executable(test_event_store test_event_store.cpp)
target_link_libraries(test_event_store PRIVATE ev_ads_core_for_test GTest::gtest_main)
gtest_discover_tests(test_event_store)

add_executable(test_model_loading test_model_loading.cpp)
target_include_directories(test_model_loading PRIVATE ${OpenCV_INCLUDE_DIRS})
target_compile_definitions(test_model_loading PRIVATE EV_ADS_PROJECT_ROOT="${CMAKE_CURRENT_LIST_DIR}/..")
target_link_libraries(test_model_loading PRIVATE ev_ads_yolo_for_test ${OpenCV_LIBS} GTest::gtest_main)
gtest_discover_tests(test_model_loading)

add_test(
  NAME project_checks
  COMMAND
    ${CMAKE_COMMAND} -E env
      "PROJECT_ROOT=${CMAKE_CURRENT_LIST_DIR}/.."
      ${Python3_EXECUTABLE} "${CMAKE_CURRENT_LIST_DIR}/scripts/project_check.py"
)
