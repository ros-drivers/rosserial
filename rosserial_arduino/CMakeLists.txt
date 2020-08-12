cmake_minimum_required(VERSION 3.7.2)
project(rosserial_arduino)

find_package(catkin REQUIRED COMPONENTS
  message_generation
)

add_message_files(FILES
  Adc.msg
)

add_service_files(FILES
  Test.srv
)

catkin_python_setup()

generate_messages()

catkin_package(
  CATKIN_DEPENDS message_runtime
  CFG_EXTRAS ${PROJECT_NAME}-extras.cmake
)

install(
  DIRECTORY src/ros_lib
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src
)

install(
  DIRECTORY arduino-cmake
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(
  PROGRAMS src/${PROJECT_NAME}/make_libraries.py nodes/serial_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
