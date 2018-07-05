# rosserial for VEX Cortex

This package contains everything needed allow you to run rosserial on the [VEX Cortex](https://www.vexrobotics.com/276-2194.html), on the [PROS Kernel](https://pros.cs.purdue.edu/cortex/index.html).

# Requirements
1. Linux (Only tested on Ubuntu 18.04LTS)
2. ROS (Only tested on ROS Melodic) - [installing ROS](http://wiki.ros.org/melodic/Installation/Source). This has been developed and tested on ROS melodic, but it should work on many earier/later versions as well.
3. PROS - [install PROS toolchain](https://github.com/purduesigbots/pros/releases/tag/2.12.1)

# Setup
Stup a ROS workspace and build rosserial packages (including rosserial_vex_cortex) from source:
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

### Step 1 : Run the ROS instance
open a terminal window and run:
```bash
source /opt/ros/melodic/setup.bash # or replace melodic with your corresponding ROS version name
roscore 
```
this will start ROS on the host linux machine.

### Step 2: get the VEX Cortex to talk
This step generates a PROS project with the necessary includes for ROS to talk to the VEX Cortex, and then downloads the "Hello World" program onto the Cortex.
Open another terminal window and run:
```bash
source <your workspace name>/install/setup.bash
cd /anywhere/on/your/computer
# below runs the generate command to build the PROS project with the rosserial libraries installed.
rosrun rosserial_vex_cortex genproject.sh <pros-project-name>
cd <pros-project-name>
pros make upload # uploads the hello world demo onto the cortex.
```
The Cortex is now trying to send messages on the "chatter" topic, as long as it is plugged into the VEX Programming cable. see `src/opcontrol.cpp` and look around so see how the PROS side works.

### Step 3: Begin the serial node!
This is how rosserial clients are able to communicate with the ROS system.
```bash
source <your-workspace-name>/install/setup.bash
rosrun rosserial_arduino serial_node.py _port:=/dev/ttyACM0 _baud:=115200
```

### Step 4: Display a topic in a terminal (technically optional)
open another terminal window and run:
```bash
source /opt/ros/melodic/setup.bash # or replace melodic with your corresponding ROS version name
rostopic echo chatter
```
this will display any messages that come through on the "chatter" topic.

If everyting is working properly, then the terminal from step 2 should show outputs of `"Hello World!"` This means the bridge between the cortex and ROS is established.
You are now able to use ROS with the VEX Cortex!
