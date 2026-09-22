cmake_minimum_required(VERSION 3.22.0)
set(lib_name google_logger)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(glog CONFIG REQUIRED)

add_library(${lib_name}_lib INTERFACE)

target_include_directories(${lib_name}_lib INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/inc)
target_link_libraries(${lib_name}_lib INTERFACE glog::glog)
target_compile_features(${lib_name}_lib INTERFACE cxx_std_20)
