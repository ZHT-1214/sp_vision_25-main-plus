cmake_minimum_required(VERSION 3.22.0)
set(lib_name function)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(GLOB_RECURSE function_src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c
)

add_library(${lib_name}_lib SHARED ${function_src})

target_include_directories(${lib_name}_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/inc)
target_link_libraries(${lib_name}_lib PUBLIC foxglove_interface_lib)
