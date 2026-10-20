cmake_minimum_required(VERSION 3.16)
project(task3_official_state_bridge LANGUAGES CXX)

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  message(FATAL_ERROR "The unchanged official runtime requires Ubuntu Linux.")
endif()

# Installed by the supplied, unmodified official Ubuntu .deb packages.
# Never generate a replacement protobuf definition or copy SDK internals.
find_package(eCAL 5.13 REQUIRED CONFIG)
find_package(Protobuf REQUIRED)
find_package(Threads REQUIRED)
find_path(OFFICIAL_ROBOT_SDK_INCLUDE_DIR NAMES robot_sdk.pb.h REQUIRED)
find_library(OFFICIAL_ROBOT_SDK_LIBRARY NAMES robot_sdk REQUIRED)

add_executable(task3_official_state_bridge official_state_bridge.cpp)
target_compile_features(task3_official_state_bridge PRIVATE cxx_std_17)
target_compile_options(task3_official_state_bridge PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(task3_official_state_bridge PRIVATE
  "${OFFICIAL_ROBOT_SDK_INCLUDE_DIR}")
target_link_libraries(task3_official_state_bridge PRIVATE
  eCAL::core protobuf::libprotobuf Threads::Threads
  "${OFFICIAL_ROBOT_SDK_LIBRARY}")
