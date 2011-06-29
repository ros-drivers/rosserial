cmake_minimum_required(VERSION 2.6)

include_directories(${ARDUINO_SDK_PATH}/hardware/arduino/cores/arduino)

set(CMAKE_MODULE_PATH    ${rosserial_arduino_PACKAGE_PATH}/cmake_scripts/modules)  # CMake module search path
#set(CMAKE_TOOLCHAIN_FILE ${rosserial_arduino_PACKAGE_PATH}/cmake_scripts/toolchains/Arduino.cmake) # Arduino Toolchain
include(${rosserial_arduino_PACKAGE_PATH}/cmake_scripts/toolchains/Arduino.cmake) # Arduino Toolchain

find_package(Arduino)

execute_process(COMMAND cp -r ${rosserial_arduino_PACKAGE_PATH}/src/ros_lib/ ${PROJECT_SOURCE_DIR}/src/)
include_directories(${PROJECT_SOURCE_DIR}/src/ros_lib)

if(ROSSERIAL_CUSTOM_SERIAL)
    set(ROS_SRCS ${rosserial_arduino_PACKAGE_PATH}/cmake_scripts/cc_support.cpp
    			   ${ROSSERIAL_CUSTOM_SERIAL})
    execute_process(COMMAND cp -rf ${rosserial_arduino_PACKAGE_PATH}/cmake_scripts/serial_fx.h ${PROJECT_SOURCE_DIR}/src/ros_lib/serial_fx.h)
    execute_process(COMMAND cp -rf ${rosserial_arduino_PACKAGE_PATH}/cmake_scripts/serial_fx.cpp ${PROJECT_SOURCE_DIR}/src/ros_lib/serial_fx.cpp)

 else()
     set(ROS_SRCS ${rosserial_arduino_PACKAGE_PATH}/cmake_scripts/cc_support.cpp)
endif()

set(ROS_SRCS ${ROS_SRCS} ${PROJECT_SOURCE_DIR}/src/ros_lib/ros_lib.cpp
						 ${PROJECT_SOURCE_DIR}/src/ros_lib/duration.cpp
						 ${PROJECT_SOURCE_DIR}/src/ros_lib/time.cpp)

			  
foreach(msg ${ROS_MSGS_USED})
string(REPLACE "/"  ";" MSG_LIST ${msg} )
LIST(GET MSG_LIST 0 MSG_PKG)
LIST(GET MSG_LIST 1 MSG_TYPE)

message(STATUS "The message pkg  ${MSG_PKG}  type: ${MSG_TYPE} is being generated")
execute_process(COMMAND rosrun rosserial_arduino make_library.py ${PROJECT_SOURCE_DIR}/src ${MSG_PKG})
endforeach(msg)

add_custom_target(clean 
    COMMAND rm -rf *.hex *.eep CMakeCache.txt cmake_install.cmake CMakeFiles *.a
)

