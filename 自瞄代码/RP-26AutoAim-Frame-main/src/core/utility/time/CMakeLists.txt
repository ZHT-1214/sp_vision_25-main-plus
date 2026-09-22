cmake_minimum_required(VERSION 3.22.0)
set(lib_name time)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(${lib_name}_lib INTERFACE)

target_include_directories(${lib_name}_lib INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/inc)
