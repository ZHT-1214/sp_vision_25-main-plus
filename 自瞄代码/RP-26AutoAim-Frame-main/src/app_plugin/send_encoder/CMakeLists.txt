set(SEND_ENCODER_PLUGIN_TARGET send_encoder_plugin_lib)
add_library(${SEND_ENCODER_PLUGIN_TARGET} SHARED src/MinimalSendEncoder.cpp)
target_include_directories(${SEND_ENCODER_PLUGIN_TARGET} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(${SEND_ENCODER_PLUGIN_TARGET} PUBLIC app_interface_lib class_loader_lib google_logger_lib)
