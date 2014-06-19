cmake_minimum_required(VERSION 2.8.3)
project(rosserial_test)

find_package(catkin REQUIRED COMPONENTS roscpp rosserial_client rosserial_python rosserial_server rostest)

catkin_package()

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

# Declare a cpp executable
add_executable(${PROJECT_NAME}_publish_subscribe src/publish_subscribe)
add_dependencies(${PROJECT_NAME}_publish_subscribe ${PROJECT_NAME}_rosserial_lib)
target_link_libraries(${PROJECT_NAME}_publish_subscribe ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

add_rostest(test/rosserial_server_socket.test)
add_rostest(test/rosserial_server_serial.test)
add_rostest(test/rosserial_python_socket.test)
add_rostest(test/rosserial_python_serial.test)
