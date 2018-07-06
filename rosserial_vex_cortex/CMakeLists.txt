cmake_minimum_required(VERSION 2.8.3)
project(rosserial_vex_cortex)

find_package(catkin REQUIRED COMPONENTS
  message_generation
)

# put the names of custom message files here
# add_message_files(FILES
#
# )

# put the names of custom service files here
# add_service_files(FILES
# 
# )

catkin_python_setup()

# uncomment once you include some messages
# generate_messages()

catkin_package(
  CATKIN_DEPENDS message_runtime
)

install(
  DIRECTORY src/ros_lib
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src
)

install(
  PROGRAMS scripts/genproject.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(
  PROGRAMS src/${PROJECT_NAME}/make_libraries.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
