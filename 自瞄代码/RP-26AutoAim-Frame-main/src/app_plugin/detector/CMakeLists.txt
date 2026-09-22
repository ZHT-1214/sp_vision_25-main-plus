set(lib_name NNdetect)
add_library(${lib_name}_lib SHARED src/NNDetector.cpp)
target_include_directories(${lib_name}_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(${lib_name}_lib PRIVATE app_interface_lib class_loader_lib google_logger_lib)
