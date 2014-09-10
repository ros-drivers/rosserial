cmake_minimum_required(VERSION 2.8.3)
project(rosserial_client)

find_package(catkin REQUIRED)
catkin_package(
  CFG_EXTRAS rosserial_client-extras.cmake
)

catkin_python_setup()

install(DIRECTORY src/ros_lib
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src
)
install(PROGRAMS scripts/make_libraries
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
