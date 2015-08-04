## rosserial_mbed

This is a rosserial client implementation for the mbed platform.

Note: This has been tested and currently supports building using the [gcc4mbed](https://github.com/adamgreen/gcc4mbed) ofline compiler for the LPC1768.

### Issues

* No support for the mbed online compiler (WIP)
* As per [issue 37](https://github.com/adamgreen/gcc4mbed/issues/37), gcc4mbed doesn't support to include srcs from an external dir.

### Usage/Workflow

* Make sure you have the gcc4mbed installed on your system
* Install the library inside your project folder
    * `rosrun rosserial_mbed make_libraries.py <project-dir>`
    * To install it to one of the examples: `rosrun rosserial_mbed make_libraries.py <rosserial_mbed_dir>/examples/Blink`
* Move to the project's folder and modify the makefile to use the correct `GCC4MBED_DIR` path
* `make all && make deploy`

Please see the [rosserial Tutorials on the ROS wiki](http://wiki.ros.org/rosserial_arduino/Tutorials) to get started.
