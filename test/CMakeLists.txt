cmake_minimum_required(VERSION 3.16)
project(ev_ads_mac_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(SQLite3 REQUIRED)

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

add_executable(test_common_and_fusion test_common_and_fusion.cpp)
target_link_libraries(test_common_and_fusion PRIVATE ev_ads_core_for_test)

add_executable(test_event_store test_event_store.cpp)
target_link_libraries(test_event_store PRIVATE ev_ads_core_for_test)

enable_testing()
add_test(NAME common_and_fusion COMMAND test_common_and_fusion)
add_test(NAME event_store COMMAND test_event_store)
