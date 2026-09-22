set(lib_name img_viz)

find_package(OpenCV REQUIRED COMPONENTS core highgui)

add_library(${lib_name}_lib SHARED
    src/img_viz.cpp
)

target_compile_features(${lib_name}_lib PUBLIC cxx_std_20)

target_include_directories(${lib_name}_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

target_link_libraries(${lib_name}_lib PUBLIC
    ${OpenCV_LIBS}
)
