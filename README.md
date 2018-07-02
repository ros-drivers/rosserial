# rosserial for VEX Cortex

[![Build Status](https://travis-ci.org/ros-drivers/rosserial.svg?branch=jade-devel)](https://travis-ci.org/ros-drivers/rosserial)

Please see [rosserial on the ROS wiki](http://wiki.ros.org/rosserial) to get started.

This package contains everything needed to run rosserial on the [VEX Cortex](https://www.vexrobotics.com/276-2194.html).

# Project Structure
rosserial_cortex provides the script `make_libraries.py`, which will generate c++ message header files that 
can be inserted into a PROS project and used to send/recieve messages in a PROS project. An example PROS project is included in this package.

# Setup
1. Install ROS
2. git clone this repository

# Usage Example(Linux)
This will show you how to run the "hello world" example PROS project.

1. Running a ROS instance:
  - open a terminal window and run:
    ```bash
    source /opt/ros/melodic/setup.bash # or replace melodic with your corresponding ROS version name
    roscore 
    ```
  - this will start ROS on the host linux machine.

2. Display a topic in a terminal (technically optional)
  - open another terminal window and run:
    ```bash
    source /opt/ros/melodic/setup.bash # or replace melodic with your corresponding ROS version name
    rostopic echo chatter
    ```
  - this will display any messages that come through on the "chatter" topic.

3. Get Cortex talking!
  - open another terminal window and run:
    ```bash
    <source workspace>
    cd <project folder>
    pros make clean
    pros make upload
    rosrun rosserial_arduino serial_node.py _port:=/dev/ttyACM0 _baud:=/115200
    ```
  - this will get the Cortex to send messages on the "chatter" topic. see `src/opcontrol.cpp` and look around so see how the PROS side works.

