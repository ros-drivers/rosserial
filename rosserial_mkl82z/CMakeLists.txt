cmake_minimum_required(VERSION 2.8.3)
project(rosserial_mkl82z)

find_package(catkin REQUIRED)

catkin_package()

include_directories()

install(DIRECTORY src/ros_lib
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src
  )
