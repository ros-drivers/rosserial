cmake_minimum_required(VERSION 2.8.3)

@[if DEVELSPACE]@
set(ROS_TEENSY_TOOLCHAIN "@(CMAKE_CURRENT_SOURCE_DIR)/teensy-cmake/teensy-arm.toolchain.cmake")
@[else]@
set(ROS_TEENSY_TOOLCHAIN "${ros_teensy_DIR}/../teensy-cmake/teensy-arm.toolchain.cmake")
@[end if]@

