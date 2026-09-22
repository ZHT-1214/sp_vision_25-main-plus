set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(GLOB_RECURSE FOXGLOVE_VIZ_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c
)

add_library(foxglove_viz SHARED ${FOXGLOVE_VIZ_SRC})

target_include_directories(foxglove_viz PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

target_link_libraries(foxglove_viz PUBLIC
    foxglove_lib
    time_lib
)
