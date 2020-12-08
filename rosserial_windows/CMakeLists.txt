cmake_minimum_required(VERSION 3.7.2)
project(rosserial_windows)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rosserial_client
  sensor_msgs
  std_msgs
)
catkin_package(CATKIN_DEPENDS)

install(
  DIRECTORY src/ros_lib
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src
)

catkin_install_python(
  PROGRAMS src/${PROJECT_NAME}/make_libraries.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
