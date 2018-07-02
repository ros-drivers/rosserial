# rosserial for VEX Cortex

This package contains everything needed allow you to do the following:
1. Run rosserial on the [VEX Cortex](https://www.vexrobotics.com/276-2194.html): send messages to/from a host Linux machine.
2. Generate custom messages: design exactly what messages you want, or use pre-defined standard ROS messages.

rosserial_cortex provides:
1. The script `make_libraries.py`, which will generate c++ files that can be inserted into a PROS project and used to send/recieve messages in a PROS project.
2. An example PROS project, to get you started on using ROS with VEX.

# Setup
1. Install [ROS](http://wiki.ros.org/melodic/Installation/Source). This has been developed and tested on ROS melodic, but it should work on many earier/later versions as well.
2. setup a workspace:
  ```bash
  mkdir -p <your-workspace-name>/src
  cd <your-workspace-name>/src
  git clone https://github.com/CanyonTurtle/rosserial.git
  cd ..
  catkin_make
  catkin_make install # this will generate folders in the workspace that contain executable scripts. 
  ```

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
    cd <your-workspace-name>
    source install/setup.bash
    cd <your-workspace-name>/src/rosserial/rosserial_cortex/src/pros-projects/working-rosserial-client # This is a PROS project folder to get you started
    cd include
    rosrun rosserial_cortex make_libraries.py . # this will generate the rosserial header files for PROS to use.
    pros make clean
    pros make upload
    rosrun rosserial_arduino serial_node.py _port:=/dev/ttyACM0 _baud:=/115200
    ```
  - this will get the Cortex to send messages on the "chatter" topic. see `src/opcontrol.cpp` and look around so see how the PROS side works.

