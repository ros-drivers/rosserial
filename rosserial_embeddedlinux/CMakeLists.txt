cmake_minimum_required(VERSION 2.8.3)
project(rosserial_embeddedlinux)

find_package(catkin REQUIRED)
catkin_package(CATKIN_DEPENDS)

install(DIRECTORY src/ros_lib
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/libraries
)

install(DIRECTORY ../rosserial_client/src/ros_lib
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/libraries
)

install(PROGRAMS src/rosserial_embeddedlinux/make_library.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
