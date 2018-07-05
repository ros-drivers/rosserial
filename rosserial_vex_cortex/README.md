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
  source /opt/ros/melodic/setup.bash # or replace melodic with your corresponding ROS version name
  mkdir -p <your-workspace-name>/src
  cd <your-workspace-name>/src
  git clone https://github.com/ros-drivers/rosserial.git
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

2. Get a Cortex talking!
  This step generates a PROS project with the necessary includes for ROS to talk to the VEX Cortex, and then downloads the "Hello World" program onto the Cortex.
  - open another terminal window and run:
    ```bash
    cd /anywhere/on/your/computer
    # below runs the generate command to build the PROS project with the rosserial libraries installed.
    /path/to/workspace/src/rosserial/rosserial_vex_cortex/src/rosserial_vex_cortex/generate.sh /path/to/workspace <project name>
    cd <project name>
    pros make upload # uploads the hello world demo onto the cortex.
    ```
  - the Cortex is now trying to send messages on the "chatter" topic, as long as it is plugged into the VEX Programming cable. see `src/opcontrol.cpp` and look around so see how the PROS side works.

3. Begin the serial node!
  This is how rosserial clients are able to communicate with the ROS system.
  ```bash
  source <your-workspace-name>/install/setup.bash
  rosrun rosserial_arduino serial_node.py _port:=/dev/ttyACM0 _baud:=115200
  ```

4. Display a topic in a terminal (technically optional)
  - open another terminal window and run:
    ```bash
    source /opt/ros/melodic/setup.bash # or replace melodic with your corresponding ROS version name
    rostopic echo chatter
    ```
  - this will display any messages that come through on the "chatter" topic.

If everyting is working properly, then the terminal from step 2 should show outputs of `"Hello World!"` This means the bridge between the cortex and ROS is established.
You are now able to use ROS with the VEX Cortex!
