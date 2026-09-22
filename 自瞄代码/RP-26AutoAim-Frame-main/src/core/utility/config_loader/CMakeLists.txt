cmake_minimum_required(VERSION 3.22.0)
set(lib_name config_loader)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED COMPONENTS core)

file(GLOB_RECURSE config_loader_src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c
)

add_library(${lib_name}_lib SHARED ${config_loader_src})

target_include_directories(${lib_name}_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/inc)
target_link_libraries(${lib_name}_lib PUBLIC opencv_core)
