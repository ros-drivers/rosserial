# rosserial for VEX V5

This package contains everything needed to run rosserial on the VEX V5
Robot Brain, on the PROS 3.x.x tooling.

# Requirements
- Software:
  1. Linux (Only tested on Ubuntu 18.04LTS) (possible on windows with a Linux 
  virtual machine, provided there is USB support)
  2. ROS installed on Linux (Only tested on ROS Melodic) - [installation guide](http://wiki.ros.org/melodic/Installation/Source).
  3. PROS installed on Linux - [installation guide](https://pros.cs.purdue.edu/v5/getting-started/index.html)
- Hardware:
  1. VEX essentials:
    - V5 Robot Brain
    - V5 Robot Battery
    - MicroUSB cable

# Setup
This setup requires knowledge of entering commands into a Linux terminal.

Note: it is possible to follow along with this guide without understanding
basic ROS constructs
(such as workspaces, projects, rosrun, roslaunch, and catkin_make).
However, in order to work with ROS beyond the examples in this project,
you will need to learn about the ROS framework itself. See the
[ROS documentation, getting-started, and tutorials pages](http://wiki.ros.org/) for more information.

### ROS Workspace
This workspace is used to generate a PROS project, and then help the Robot Brain
interact with ROS using that generated project.

Notice the first `source` command below. This includes ROS commands into the terminal
(such as catkin_make), so it will come first in
most of the terminal commands in this setup process. Make sure you replace `melodic`
with your corresponding ROS version name, if it

is not `melodic` (e.g. `kinetic`).
Open up a terminal, and enter:

```bash
source /opt/ros/melodic/setup.bash
mkdir -p ~/ros-vex-workspace/src; cd ~/ros-vex-workspace/src
git clone https://github.com/ros-drivers/rosserial.git
cd ..; catkin_make; catkin_make install
```

### PROS Project

1. Generate a PROS project. A PROS project has the code that runs on the Robot Brain.
The project can exist anywhere (it should NOT need to be inside the ROS workspace). Also see the
[getting-started documentation](https://pros.cs.purdue.edu/v5/getting-started/index.html) if you need help.

```bash
cd ~
prosv5 conductor new-project example-project-name
```

2. Download the newest ros_lib zip file template from it's [repository](https://github.com/UTAH-VEXU-Robotics/ros_lib/releases). 
Then fetch and apply the template to your project by following below.

```bash
cd example-project-name
prosv5 conductor fetch ~/Downloads/ros_lib@1.0.x.zip
prosv5 conductor apply ros_lib@1.0.x
```

# Examples

These examples are made to run out-of-the-box, and they made to be proof-of-concepts of
what ROS can provide. ROS allows standardized messages to be sent to and from the Brain and an outside computer, 
which allows for all kinds of ideas and projects to be organized with messages!

To understand what is going on with the example code, look at the tutorials for the sister project, [Rosserial Arduino](http://wiki.ros.org/rosserial_arduino/Tutorials).

Set up the physical download connection by plugging in the microUSB into the Robot Brain and into the computer.

### Hello World Example

To run the hello world example, make sure that the `hellow_world()` method in the `main.cpp` file is not commented.

After doing that, run the following commands.

```bash
source ~/ros-vex-workspace/install/setup.bash
cd ~/example-project-name
prosv5 mu --execute #this will make, upload, and execute the program to the brain
roslaunch rosserial_vex_v5 hello_world.launch #run this so that the computer can talk to the brain
```

### Sensors Example

To run the sensors example, make sure that the `main.cpp` file looks like this:

```cpp
  //hello_world();
  sensors();
  //odometry();
```

*Note: you might also want to change the code in the `example-project-name/include/ros_lib/rosserial_vex_v5/examples/sensors.hpp` file.*

After doing that, run the following commands.

```bash
source ~/ros-vex-workspace/install/setup.bash
cd ~/example-project-name
prosv5 mu --execute #this will make, upload, and execute the program to the brain
roslaunch rosserial_vex_v5 hello_world.launch #run this so that the computer can talk to the brain
```

### Odometry Example

To run the odometry example, make sure that the `main.cpp` file looks like this:

```cpp
  //hello_world();
  //sensors();
  odometry();
```

*Note: you might also want to change the code in the `example-project-name/include/ros_lib/rosserial_vex_v5/examples/odometry.hpp` file.*

After doing that, run the following commands.

```bash
source ~/ros-vex-workspace/install/setup.bash
cd ~/example-project-name
prosv5 mu --execute #this will make, upload, and execute the program to the brain
roslaunch rosserial_vex_v5 odometry.launch #run this so that the computer can talk to the brain
```

There is a [video](https://youtu.be/_WbhUeprUS8) that shows what this should look like.


# Generating Custom Messages
To design you own ROS messages, it is necessary to add the `msgs` directory to this project,
add the `message_generation` dependency, and make several modifications to this project's `CMakeLists.txt`.
This infrastructure is removed from this project by default,
because there are many useful built-in message types that are used commonly across ROS packages anyway.

See documentation and source of sister packages, such as `rosserial_arduino`,
for more information about generating custom messages.

### Platforms
This has been developed and tested on ROS melodic, but it should work on many earier/later versions as well.

# Speed
Over microUSB, any message can stream at 100hz. Higher speeds (e.g. 500hz) are unstable right now.

# Troubleshooting
If any part of the launching or downloading process fails,
try unplugging the V5 Brain and replugging it in after 20 seconds.
It is possible that the ports being used (user vs. system) got switched around,
and need to reset.

This project is similar to [Rosserial Arduino](http://wiki.ros.org/rosserial_arduino/Tutorials) in usage, so refer to these tutorials for even more information.