Rosserial client for rp2040 usage

- Install the catkin package for rosserial
- Install the rp2040 toolchain
- Create a rp2040 project
- Add the following lines to CMakeLists.txt (above target_link_libraries):
	add_subdirectory($ENV{PICO_SDK_PATH}/lib/ros_lib build)
	target_include_directories(${PROJECT_NAME} PRIVATE $ENV{PICO_SDK_PATH}/lib/ros_lib)
- Add 'ros_lib' to the target_link_libraries in CMakeLists.txt (without quotes)
- Execute the command 'rosrun rosserial_arduino make_libraries.py' to generate the ros_lib library files in the pico-sdk library folder
- To generate the ros_lib library files in a custom location use 'rosrun rosserial_arduino make_libraries.py <custom location>'

The rosserial functionality can now be accessed from the project as described in the rosserial tutorials.

