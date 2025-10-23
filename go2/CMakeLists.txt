cmake_minimum_required(VERSION 3.12)
project(go2_controller)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)

# Detect platform architecture
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
    set(ONNXRUNTIME_PLATFORM "x64")
    message(STATUS "Detected x64 platform, using onnxruntime-linux-x64-1.22.0")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(ONNXRUNTIME_PLATFORM "aarch64")
    message(STATUS "Detected ARM64 platform, using onnxruntime-linux-aarch64-1.22.0")
else()
    message(FATAL_ERROR "Unsupported platform architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# Set onnxruntime path
set(ONNXRUNTIME_ROOT ${PROJECT_SOURCE_DIR}/../thirdparty/onnxruntime-linux-${ONNXRUNTIME_PLATFORM}-1.22.0)

include_directories(
  /usr/include/eigen3
  /usr/local/include/ddscxx # dds
  /usr/local/include/iceoryx/v2.0.2 # dds
  include
  ${ONNXRUNTIME_ROOT}/include
  ${PROJECT_SOURCE_DIR}/../include/
)

link_libraries(
  unitree_sdk2 ddsc ddscxx rt pthread # dds
  libboost_program_options.a libyaml-cpp.a fmt
  ${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so
)

file(GLOB_RECURSE ADD_SRC_LIST
  src/*.cpp
)

add_library(${PROJECT_NAME}_lib ${ADD_SRC_LIST})
link_libraries(${PROJECT_NAME}_lib)

add_executable(go2_ctrl main.cpp)