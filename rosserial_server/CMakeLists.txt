cmake_minimum_required(VERSION 3.7.2)
project(rosserial_server)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosserial_msgs
  std_msgs
  topic_tools
)

find_package(Boost REQUIRED COMPONENTS
  system
  thread
)

find_package(PythonLibs REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    rosserial_msgs
    std_msgs
    topic_tools
  LIBRARIES ${PROJECT_NAME}_lookup
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_lookup src/msg_lookup.cpp)
target_link_libraries(${PROJECT_NAME}_lookup ${PYTHON_LIBRARY})
target_include_directories(${PROJECT_NAME}_lookup SYSTEM PRIVATE ${PYTHON_INCLUDE_DIR})

add_executable(${PROJECT_NAME}_serial_node src/serial_node.cpp)
target_link_libraries(${PROJECT_NAME}_serial_node ${catkin_LIBRARIES} ${PROJECT_NAME}_lookup)
set_target_properties(${PROJECT_NAME}_serial_node PROPERTIES OUTPUT_NAME serial_node PREFIX "")
add_dependencies(${PROJECT_NAME}_serial_node ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_socket_node src/socket_node.cpp)
target_link_libraries(${PROJECT_NAME}_socket_node ${catkin_LIBRARIES} ${PROJECT_NAME}_lookup)
set_target_properties(${PROJECT_NAME}_socket_node PROPERTIES OUTPUT_NAME socket_node PREFIX "")
add_dependencies(${PROJECT_NAME}_socket_node ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_udp_socket_node src/udp_socket_node.cpp)
target_link_libraries(${PROJECT_NAME}_udp_socket_node ${catkin_LIBRARIES} ${PROJECT_NAME}_lookup)
set_target_properties(${PROJECT_NAME}_udp_socket_node PROPERTIES OUTPUT_NAME udp_socket_node PREFIX "")
add_dependencies(${PROJECT_NAME}_udp_socket_node ${catkin_EXPORTED_TARGETS})

install(
  TARGETS
    ${PROJECT_NAME}_lookup
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  TARGETS
    ${PROJECT_NAME}_serial_node
    ${PROJECT_NAME}_socket_node
    ${PROJECT_NAME}_udp_socket_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
