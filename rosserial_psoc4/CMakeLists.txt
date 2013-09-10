cmake_minimum_required(VERSION 2.8.3)
project(rosserial_psoc4)

find_package(catkin REQUIRED COMPONENTS message_generation)

add_message_files(FILES
                  Adc.msg
                  Nh_diag.msg
                 )

add_service_files(FILES
                  Test.srv
                 )

generate_messages()

catkin_package(CATKIN_DEPENDS message_runtime)

install(DIRECTORY src/ros_lib
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src
)

install(PROGRAMS src/rosserial_psoc4/make_libraries.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



