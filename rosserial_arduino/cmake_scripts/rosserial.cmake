cmake_minimum_required(VERSION 2.6)

include_directories(${PROJECT_SOURCE_DIR}/src/roslib)
include_directories(${ARDUINO_SDK_PATH}/hardware/arduino/cores/arduino)

set(CMAKE_MODULE_PATH    ${rosserial_arduino_PACKAGE_PATH}/cmake_scripts/modules)  # CMake module search path
#set(CMAKE_TOOLCHAIN_FILE ${rosserial_arduino_PACKAGE_PATH}/cmake_scripts/toolchains/Arduino.cmake) # Arduino Toolchain
include(${rosserial_arduino_PACKAGE_PATH}/cmake_scripts/toolchains/Arduino.cmake) # Arduino Toolchain

find_package(Arduino)

execute_process(COMMAND cp -r ${rosserial_arduino_PACKAGE_PATH}/src/roslib ${PROJECT_SOURCE_DIR}/src/roslib)
set(ros_SRCS ${rosserial_arduino_PACKAGE_PATH}/cmake_scripts/cc_support.cpp
			  ${PROJECT_SOURCE_DIR}/src/roslib/ros_lib.cpp )

#Add the target for generating the rosserial libraries for arduino
message("Generating Messages ${ROS_MSGS_USED}")

foreach(msg ${ROS_MSGS_USED})
string(REPLACE "/"  ";" MSG_LIST ${msg} )
LIST(GET MSG_LIST 0 MSG_PKG)
LIST(GET MSG_LIST 1 MSG_TYPE)

message(STATUS "The message pkg  ${MSG_TYPE}  type: ${MSG_TYPE} is being generated .")
execute_process(COMMAND rosrun rosserial_arduino make_library.py ${PROJECT_SOURCE_DIR}/src/roslib ${MSG_PKG})

set(ros_SRCS ${ros_SRCS} ${PROJECT_SOURCE_DIR}/src/roslib/${MSG_PKG}_${MSG_TYPE}.cpp)

endforeach(msg)

                        
file(GLOB_RECURSE ros_HDRS 
                                ${PROJECT_SOURCE_DIR}/src/roslib/*.h
                                ${PROJECT_SOURCE_DIR}/src/roslib/*/*.h)
set(ros_BOARD atmegea328p)

generate_arduino_library(ros)
