cmake_minimum_required(VERSION 2.8.3)
project(rosserial_embeddedlinux)

find_package(catkin REQUIRED)
catkin_package(CATKIN_DEPENDS)

#add_custom_target( make_libraries ALL
#    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/../rosserial_client/src/rosserial_client/make_library.py ${CATKIN_DEVEL_PREFIX}/#libraries std_msgs diagnostic_msgs geometry_msgs nav_msgs sensor_msgs rosserial_msgs tf
#)

#install(DIRECTORY libraries
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

install(DIRECTORY src/ros_lib
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/libraries
)

install(DIRECTORY ../rosserial_client/src/ros_lib
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/libraries
)
