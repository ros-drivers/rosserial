cmake_minimum_required(VERSION 3.7.2)
project(rosserial_python)

find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

catkin_install_python(
  PROGRAMS nodes/message_info_service.py nodes/serial_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
