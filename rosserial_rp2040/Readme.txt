Rosserial client for rp2040 usage

- Install the catkin package for rosserial
- Install the rp2040 toolchain
- Create a rp2040 project
- Add the following lines to CMakeLists.txt (above target_link_libraries)
	add_subdirectory(ros_lib)
	target_include_directories(${PROJECT_NAME} PRIVATE ros_lib)
- Add ros_lib to the target_link_libraries

The rosserial functionality can now be accessed from the project as described in the rosserial tutorials.

