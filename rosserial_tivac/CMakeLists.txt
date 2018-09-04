cmake_minimum_required(VERSION 2.8.3)
project(rosserial_tivac)

find_package(catkin REQUIRED)

catkin_package(
  CFG_EXTRAS ${PROJECT_NAME}-extras.cmake
)

install(
  DIRECTORY
    src/ros_lib
    src/ros_lib_energia
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src
)

install(
  DIRECTORY tivac-cmake
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  PROGRAMS
    src/${PROJECT_NAME}/make_libraries_energia
    src/${PROJECT_NAME}/make_libraries_tiva
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
