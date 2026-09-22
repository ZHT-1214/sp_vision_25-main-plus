cmake_minimum_required(VERSION 3.22.0)
set(lib_name transform_tools)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED COMPONENTS core calib3d)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)

file(GLOB_RECURSE transform_tools_lib_source
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c
)

add_library(${lib_name}_lib SHARED ${transform_tools_lib_source})

target_include_directories(${lib_name}_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(${lib_name}_lib PUBLIC
    ${OpenCV_LIBS}
    Eigen3::Eigen
    Sophus::Sophus
)
