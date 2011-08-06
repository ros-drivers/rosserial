cmake_minimum_required(VERSION 2.6)

#lets set the name of the project to the name of the pkg
#this emulates the typical rosbuild
get_filename_component(_project ${CMAKE_SOURCE_DIR} NAME)
project(${_project})

set(CMAKE_MODULE_PATH    ${rosserial_arduino_PACKAGE_PATH}/cmake_scripts/modules)  # CMake module search path
include(${rosserial_arduino_PACKAGE_PATH}/cmake_scripts/toolchains/Arduino.cmake) # Arduino Toolchain

find_package(Arduino)

rosbuild_find_ros_package(rosserial_client)
execute_process(COMMAND cp -r ${rosserial_client_PACKAGE_PATH}/src/ros_lib/ ${PROJECT_SOURCE_DIR}/src/)
execute_process(COMMAND cp -r ${rosserial_arduino_PACKAGE_PATH}/src/ros_lib/ ${PROJECT_SOURCE_DIR}/src/)
include_directories(${PROJECT_SOURCE_DIR}/src/ros_lib)


#get a list of all the pkgs in the manifest
#so that we can generate the rosserial implementation for them
rosbuild_invoke_rospack(${PROJECT_NAME} ${PROJECT_NAME} "depedencies" "depends")
string(REGEX REPLACE "\n" ";" ${PROJECT_NAME}_depedencies ${${PROJECT_NAME}_depedencies})

foreach(MSG_PKG ${${PROJECT_NAME}_depedencies})
rosbuild_find_ros_package(${MSG_PKG})

	if (EXISTS ${${MSG_PKG}_PACKAGE_PATH}/msg AND 
		NOT EXISTS ${PROJECT_SOURCE_DIR}/src/ros_lib/${MSG_PKG})
		message(STATUS "Generating rosserial implementation for ${MSG_PKG} in ${PROJECT_SOURCE_DIR}/src" )
		execute_process(COMMAND rosrun rosserial_client make_library.py ${PROJECT_SOURCE_DIR}/src ${MSG_PKG} OUTPUT_QUIET)
	endif()
endforeach(MSG_PKG)

SET_DIRECTORY_PROPERTIES(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES 
     ${PROJECT_SOURCE_DIR}/src/ros_lib
)



# - Generate firmware for rosserial_arduino Devices
# generate_ros_firmware(TARGET_NAME)
#        TARGET_NAME - Name of target
# Creates a Arduino firmware target.
#
# The target options can be configured by setting options of
# the following format:
#      ${TARGET_NAME}${SUFFIX}
# The following suffixes are availabe:
#      _SRCS           # Sources
#      _HDRS           # Headers
#      _LIBS           # Libraries to linked in
#      _BOARD          # Board name (such as uno, mega2560, ...)
#      _PORT           # Serial port, for upload and serial targets [OPTIONAL]
#      _AFLAGS         # Override global Avrdude flags for target
#      _SERIAL         # Serial command for serial target           [OPTIONAL]
#      _NO_AUTOLIBS    # Disables Arduino library detection
#      _NO_DEFAULT_COM # Disables a default communication implementation
#
# Here is a short example for a target named test:
#       set(test_SRCS  test.cpp)
#       set(test_HDRS  test.h)
#       set(test_BOARD uno)
#       generate_ros_firmware(test)

macro(generate_ros_firmware TARGET_NAME)

	FILE(GLOB ROS_SRCS		${PROJECT_SOURCE_DIR}/src/ros_lib/*.cpp)


	set(ROS_SRCS ${ROS_SRCS} 
	      ${rosserial_arduino_PACKAGE_PATH}/cmake_scripts/cc_support.cpp)


	#add in ROS SRCS
	set(${TARGET_NAME}_SRCS ${${TARGET_NAME}_SRCS} ${ROS_SRCS})
	generate_arduino_firmware(${TARGET_NAME})

	SET_DIRECTORY_PROPERTIES(PROPERTIES  ADDITIONAL_MAKE_CLEAN_FILES 
			"${PROJECT_SOURCE_DIR}/${TARGET_NAME}.eep;${PROJECT_SOURCE_DIR}/${TARGET_NAME}.hex")
endmacro()
