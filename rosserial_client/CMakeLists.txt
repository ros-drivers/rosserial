cmake_minimum_required(VERSION 2.8.3)
project(rosserial_client)

find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY src/ros_lib
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS src/rosserial_client/make_library.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
