cmake_minimum_required(VERSION 2.8.3)
project(rosserial_test)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rosserial_client rosserial_python rosserial_server rostest)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rosserial_test
#  CATKIN_DEPENDS gtest roscpp rosserial_client rosserial_python rosserial_server rostest
#  DEPENDS system_lib
)

###########
## Build ##
###########

# Generate a client library for the test harnesses to use.
add_custom_command(
  OUTPUT ${PROJECT_BINARY_DIR}/include/rosserial
  COMMAND rosrun rosserial_test generate_client_ros_lib ${PROJECT_BINARY_DIR}/include
)
add_custom_target(${PROJECT_NAME}_rosserial_lib DEPENDS ${PROJECT_BINARY_DIR}/include/rosserial)
add_dependencies(${PROJECT_NAME}_rosserial_lib std_msgs_genpy rosserial_msgs_genpy)

include_directories(
  include ${PROJECT_BINARY_DIR}/include ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(rosserial_test
#   src/${PROJECT_NAME}/rosserial_test.cpp
# )

## Declare a cpp executable
add_executable(${PROJECT_NAME}_publish_subscribe src/publish_subscribe)
add_dependencies(${PROJECT_NAME}_publish_subscribe ${PROJECT_NAME}_rosserial_lib)
target_link_libraries(${PROJECT_NAME}_publish_subscribe gtest util ${catkin_LIBRARIES})
add_rostest(test/rosserial_server_socket.test)
add_rostest(test/rosserial_python_socket.test)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS rosserial_test rosserial_test_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rosserial_test.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
