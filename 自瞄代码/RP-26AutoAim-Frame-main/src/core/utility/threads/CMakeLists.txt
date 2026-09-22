#与数据结构以及线程和相关模块的cmakelists
set(THREADS_TARGET threads_lib)


#收集源文件SENSORS_SRC（无需收集）


#生成INTERFACE库,指定编译标准
add_library(${THREADS_TARGET} INTERFACE)


#头文件路径
target_include_directories(${THREADS_TARGET} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/inc)#自己的头文件


#链接库(无需链接)
