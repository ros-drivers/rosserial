cmake_minimum_required(VERSION 3.7.2)
project(rosserial_chibios)

find_package(catkin REQUIRED)

catkin_package()

install(
  DIRECTORY src/ros_lib
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src
)

catkin_install_python(
  PROGRAMS src/${PROJECT_NAME}/make_libraries.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
