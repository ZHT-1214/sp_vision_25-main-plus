
find_package(OpenCV REQUIRED COMPONENTS core)

add_library(app_interface_lib INTERFACE)
target_include_directories(app_interface_lib INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/interface
    ${CMAKE_CURRENT_SOURCE_DIR}/context
)
target_link_libraries(app_interface_lib INTERFACE threads_lib)
target_link_libraries(app_interface_lib INTERFACE time_lib)
target_link_libraries(app_interface_lib INTERFACE opencv_core)
target_link_libraries(app_interface_lib INTERFACE transform_tools_lib)





add_executable(app runtime/main.cpp)
target_link_libraries(app PRIVATE app_interface_lib)


file(GLOB_RECURSE
    CLASSLOADER_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/class_loader/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/class_loader/*.c
)
add_library(class_loader_lib SHARED ${CLASSLOADER_SOURCES})
target_include_directories(class_loader_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/class_loader)
target_link_libraries(class_loader_lib PUBLIC app_interface_lib PRIVATE ${CMAKE_DL_LIBS})

target_link_libraries(app PRIVATE class_loader_lib google_logger_lib opencv_core img_viz_lib)
