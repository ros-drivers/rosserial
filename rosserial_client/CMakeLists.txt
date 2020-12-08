cmake_minimum_required(VERSION 3.7.2)
project(rosserial_client)

find_package(catkin REQUIRED)
catkin_package(
  CFG_EXTRAS ${PROJECT_NAME}-extras.cmake
)

catkin_python_setup()

if(CATKIN_ENABLE_TESTING)
  find_package(rosserial_msgs REQUIRED)
  include_directories(src/ros_lib ${rosserial_msgs_INCLUDE_DIRS})
  catkin_add_gtest(float64_test test/float64_test.cpp)
  catkin_add_gtest(subscriber_test test/subscriber_test.cpp)
  catkin_add_gtest(time_test test/time_test.cpp src/ros_lib/duration.cpp src/ros_lib/time.cpp)
endif()

install(
  DIRECTORY src/ros_lib
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src
)

catkin_install_python(
  PROGRAMS scripts/make_libraries src/${PROJECT_NAME}/make_library.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
