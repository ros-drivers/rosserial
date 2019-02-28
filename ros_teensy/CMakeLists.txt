cmake_minimum_required(VERSION 2.8.3)
project(ros_teensy)

find_package(catkin REQUIRED COMPONENTS message_generation)

catkin_package(
  CATKIN_DEPENDS message_runtime
  CFG_EXTRAS extras.cmake
)

install(DIRECTORY teensy-cmake
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

