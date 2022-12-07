Rosserial client for rp2040 usage

- Install the catkin package for rosserial
- Install the rp2040 toolchain
- Create a rp2040 project
- Add the following lines to CMakeLists.txt (above target_link_libraries):
	add_subdirectory(ros_lib)
	target_include_directories(${PROJECT_NAME} PRIVATE ros_lib)
- Add 'ros_lib' to the target_link_libraries in CMakeLists.txt (without quotes)
- Execute the command 'rosrun rosserial_arduino make_libraries.py .' in the source folder of the project to create the 'ros_lib' folder

The rosserial functionality can now be accessed from the project as described in the rosserial tutorials.

