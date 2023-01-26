cmake_minimum_required(VERSION 3.13)

project(ros_lib)

add_library(ros_lib STATIC
    duration.cpp
    time.cpp
)

target_include_directories(ros_lib PRIVATE .)

install(TARGETS ros_lib DESTINATION lib)
install(FILES ros.h DESTINATION include)
