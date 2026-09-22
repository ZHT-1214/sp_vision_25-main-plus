function(awakening_enable_pch target_name)
    if(BUILD_WITH_PCH AND COMMAND target_precompile_headers)
        target_precompile_headers(
            ${target_name}
            PRIVATE "$<$<COMPILE_LANGUAGE:CXX>:${AWAKENING_PCH_HEADER}>"
        )
    endif()
endfunction()

function(awakening_add_library target_name)
    set(options EXCLUDE_FROM_ALL)
    set(one_value_args TYPE)
    set(multi_value_args
        SOURCES
        GLOB
        INCLUDES
        DEFINITIONS
        LINKS
        PUBLIC_LINKS
        PRIVATE_LINKS
    )
    cmake_parse_arguments(AW_LIB "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})

    if(NOT AW_LIB_TYPE)
        set(AW_LIB_TYPE SHARED)
    endif()

    set(target_sources ${AW_LIB_SOURCES})
    if(AW_LIB_GLOB)
        file(GLOB_RECURSE glob_sources CONFIGURE_DEPENDS ${AW_LIB_GLOB})
        list(APPEND target_sources ${glob_sources})
    endif()

    if(AW_LIB_EXCLUDE_FROM_ALL)
        add_library(${target_name} ${AW_LIB_TYPE} EXCLUDE_FROM_ALL ${target_sources})
    else()
        add_library(${target_name} ${AW_LIB_TYPE} ${target_sources})
    endif()

    awakening_enable_pch(${target_name})

    if(AW_LIB_INCLUDES)
        target_include_directories(${target_name} PUBLIC ${AW_LIB_INCLUDES})
    endif()

    if(AW_LIB_DEFINITIONS)
        target_compile_definitions(${target_name} PUBLIC ${AW_LIB_DEFINITIONS})
    endif()

    if(AW_LIB_PUBLIC_LINKS OR AW_LIB_PRIVATE_LINKS)
        if(AW_LIB_LINKS OR AW_LIB_PUBLIC_LINKS)
            target_link_libraries(${target_name} PUBLIC ${AW_LIB_LINKS} ${AW_LIB_PUBLIC_LINKS})
        endif()
        if(AW_LIB_PRIVATE_LINKS)
            target_link_libraries(${target_name} PRIVATE ${AW_LIB_PRIVATE_LINKS})
        endif()
    elseif(AW_LIB_LINKS)
        target_link_libraries(${target_name} ${AW_LIB_LINKS})
    endif()
endfunction()

function(awakening_add_executable exe_name)
    set(options TOOL EXCLUDE_FROM_ALL)
    set(multi_value_args SOURCES LINKS PRIVATE_LINKS)
    cmake_parse_arguments(AW_EXE "${options}" "" "${multi_value_args}" ${ARGN})

    set(exclude_from_all FALSE)
    if(AW_EXE_EXCLUDE_FROM_ALL OR (AW_EXE_TOOL AND NOT BUILD_RUNTIME_TESTS))
        set(exclude_from_all TRUE)
    endif()

    if(exclude_from_all)
        add_executable(${exe_name} EXCLUDE_FROM_ALL ${AW_EXE_SOURCES})
    else()
        add_executable(${exe_name} ${AW_EXE_SOURCES})
    endif()

    awakening_enable_pch(${exe_name})

    set(default_links ${PROJECT_NAME}_utils relink)
    if(RERUN_FOUND)
        list(APPEND default_links ${PROJECT_NAME}_rerun)
    endif()

    if(AW_EXE_PRIVATE_LINKS)
        target_link_libraries(${exe_name} ${default_links} ${AW_EXE_LINKS})
        target_link_libraries(${exe_name} PRIVATE ${AW_EXE_PRIVATE_LINKS})
    else()
        target_link_libraries(${exe_name} ${default_links} ${AW_EXE_LINKS})
    endif()
endfunction()

function(add_common_executable exe_name src_file)
    awakening_add_executable(${exe_name} SOURCES ${src_file} LINKS ${ARGN})
endfunction()

function(add_tool_executable exe_name src_file)
    awakening_add_executable(${exe_name} TOOL SOURCES ${src_file} LINKS ${ARGN})
endfunction()
