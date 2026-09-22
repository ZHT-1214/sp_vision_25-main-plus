cmake_minimum_required(VERSION 3.10)
project(buff_algorithm)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# 允许本库被链接进其它项目的共享库中（例如作为 ROS2 组件节点库的一部分）。
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_compile_options(-O3 -Wall -Wextra)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# TensorRT + CUDA：用于 buff.engine 的推理，必需。
set(TENSORRT_ROOT "" CACHE PATH "TensorRT installation root (contains include/ and lib/)")
find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  HINTS
    ${TENSORRT_ROOT}/include
  PATHS
    /usr/include/x86_64-linux-gnu
    /usr/local/tensorrt/include
)
find_library(TENSORRT_LIBRARY NAMES nvinfer
  HINTS
    ${TENSORRT_ROOT}/lib
  PATHS
    /usr/lib/x86_64-linux-gnu
    /usr/local/tensorrt/lib
)
find_path(CUDA_INCLUDE_DIR cuda_runtime_api.h
  HINTS
    ${TENSORRT_ROOT}/include
  PATHS
    /usr/local/cuda/include
    /usr/local/cuda-12/include
    /usr/local/cuda-13/include
    /usr/include/x86_64-linux-gnu
)
find_library(CUDART_LIBRARY NAMES cudart
  HINTS
    ${TENSORRT_ROOT}/lib
  PATHS
    /usr/local/cuda/lib64
    /usr/local/cuda-12/lib64
    /usr/local/cuda-13/lib64
    /usr/lib/x86_64-linux-gnu
)

if(NOT (TENSORRT_INCLUDE_DIR AND TENSORRT_LIBRARY AND CUDA_INCLUDE_DIR AND CUDART_LIBRARY))
  message(FATAL_ERROR
    "TensorRT/CUDA not found (required for buff.engine inference); "
    "pass -DTENSORRT_ROOT=... / -DCUDA_TOOLKIT_ROOT_DIR=...")
endif()
message(STATUS
  "tensorrt: include=${TENSORRT_INCLUDE_DIR} lib=${TENSORRT_LIBRARY} cuda=${CUDA_INCLUDE_DIR}")

file(GLOB_RECURSE BUFF_ALGO_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

add_library(buff_algorithm STATIC ${BUFF_ALGO_SOURCES})

target_include_directories(buff_algorithm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(buff_algorithm PUBLIC
  ${OpenCV_LIBS}
  Eigen3::Eigen
  ${TENSORRT_LIBRARY}
  ${CUDART_LIBRARY}
)

target_include_directories(buff_algorithm PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ${TENSORRT_INCLUDE_DIR}
  ${CUDA_INCLUDE_DIR}
)

option(BUFF_ALGO_BUILD_EXAMPLES "Build buff_algorithm examples" ON)
if(BUFF_ALGO_BUILD_EXAMPLES)
  add_executable(buff_algo_example examples/example.cpp)
  target_link_libraries(buff_algo_example PRIVATE buff_algorithm)
  target_compile_definitions(buff_algo_example PRIVATE
    BUFF_ALGO_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config"
  )
endif()

# 只有在独立编译本库时才安装（作为子目录被其它项目 add_subdirectory 引入时，
# 由上层项目决定如何安装/链接，避免与上层的安装规则冲突，例如 ROS2 的
# ament_cmake_symlink_install）。
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  install(TARGETS buff_algorithm
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
  install(DIRECTORY include/ DESTINATION include)
endif()
