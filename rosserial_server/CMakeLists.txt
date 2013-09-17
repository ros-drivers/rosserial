cmake_minimum_required(VERSION 2.8.3)
project(rosserial_server)

find_package(catkin REQUIRED COMPONENTS roscpp rosserial_msgs topic_tools)
find_package(Boost REQUIRED COMPONENTS system thread)

catkin_package(
)

###########
## Build ##
###########

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_serial_node src/serial_node.cpp)
target_link_libraries(${PROJECT_NAME}_serial_node ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_serial_node PROPERTIES OUTPUT_NAME serial_node PREFIX "")

add_executable(${PROJECT_NAME}_socket_node src/socket_node.cpp)
target_link_libraries(${PROJECT_NAME}_socket_node ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_socket_node PROPERTIES OUTPUT_NAME socket_node PREFIX "")

#############
## Install ##
#############

install(
  TARGETS ${PROJECT_NAME}_socket_node ${PROJECT_NAME}_serial_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rosserial_server.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
