cmake_minimum_required(VERSION 3.7.2)
project(rosserial_xbee)

find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

install(
  PROGRAMS scripts/setup_xbee.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  PROGRAMS scripts/xbee_network.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
