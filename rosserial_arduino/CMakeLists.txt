cmake_minimum_required(VERSION 2.8.3)
project(rosserial_arduino)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation)

add_message_files(FILES
                  Adc.msg
                  KawasakiImu.msg
                 )

add_service_files(FILES
                  Test.srv
                 )

generate_messages(DEPENDENCIES
                  std_msgs
                  )

catkin_package(CATKIN_DEPENDS message_runtime)

install(DIRECTORY src/ros_lib
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src
)

install(PROGRAMS src/rosserial_arduino/make_libraries.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



