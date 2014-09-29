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

if(CATKIN_ENABLE_TESTING)
  include_directories(src)
  catkin_add_gtest(float64_test test/float64_test.cpp)
endif()
