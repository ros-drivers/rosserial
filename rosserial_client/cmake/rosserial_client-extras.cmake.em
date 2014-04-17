cmake_minimum_required(VERSION 2.8.3)

@[if DEVELSPACE]@
set(ROSSERIAL_CLIENT_MAKE_LIBRARIES_SCRIPT "@(CMAKE_CURRENT_SOURCE_DIR)/scripts/make_libraries_generic")
@[else]@
set(ROSSERIAL_CLIENT_MAKE_LIBRARIES_SCRIPT "${rosserial_client_DIR}/../../../@(CATKIN_PACKAGE_BIN_DESTINATION)/make_libraries_generic")
@[end if]@

function(rosserial_client_make_libraries_generic)
  add_custom_command(
    OUTPUT ${PROJECT_BINARY_DIR}/ros_lib
    COMMAND ${ROSSERIAL_CLIENT_MAKE_LIBRARIES_SCRIPT} ${PROJECT_BINARY_DIR}
  )
  add_custom_target(${PROJECT_NAME}_ros_lib DEPENDS ${PROJECT_BINARY_DIR}/ros_lib)
  add_dependencies(${PROJECT_NAME}_ros_lib rosserial_msgs_genpy std_msgs_genpy)
  set(${PROJECT_NAME}_ROS_LIB_DIR "${PROJECT_BINARY_DIR}/ros_lib" PARENT_SCOPE)
endfunction()
