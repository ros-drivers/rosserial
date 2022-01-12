# rosserial for VEX Cortex

This package contains everything needed to run rosserial on the [VEX Cortex](https://www.vexrobotics.com/276-2194.html), on the [PROS Kernel](https://pros.cs.purdue.edu/cortex/index.html).

# Requirements
- Software:
  1. Linux (Only tested on Ubuntu 18.04LTS) (possible on windows with a Linux virtual machine, provided there is USB support)
  2. ROS installed on Linux (Only tested on ROS Melodic) - [installation guide](http://wiki.ros.org/melodic/Installation/Source).
  3. PROS installed on Linux - [installation guide](https://pros.cs.purdue.edu/cortex/getting-started/index.html)
- Hardware:
  1. VEX essentials:
    - VEX Cortex
    - VEX Joystick
    - VEXnet keys
    - VEX Competition battery
    - VEX Programming Cable
  2. (optional, for debugging) a [USB-serial adapter](https://www.adafruit.com/product/954) 
  3. (optional, for debugging) Three Male-Male jumper wires for USB-serial adapter
 
# Table Of Contents
- [Setup](#setup)
- [Examples](#examples)
  - [Hello World Example](#hello-world-example)
  - [Keyboard or Android Phone Driving Example](#keyboard-or-android-driving-example)
  - [Alternative Joystick Example](#alternative-joystick-example)
- [Physical Serial Connections](#physical-serial-connections)
- [Generating Custom Messages](#generating-custom-messages)
- [Limitations](#limitations)
- [Speed](#speed)
- [Troubleshooting](#troubleshooting)

# Setup
This setup requires knowledge of entering commands into a Linux terminal.

Note: it is possible to follow along with this guide without understanding basic ROS constructs
(such as workspaces, projects, rosrun, roslaunch, and catkin_make). However, in order to work
with ROS beyond the examples in this project, you will need to learn about the ROS framework itself. See the
[ROS documentation, getting-started, and tutorials pages](http://wiki.ros.org/) for more information.

### ROS Workspace
This workspace is used to generate a PROS project, and then help the Cortex interact with ROS using that generated project.

Notice the first `source` command below. This includes ROS commands into the terminal (such as catkin_make), so it will come first in
most of the terminal commands in this setup process. Make sure you replace `melodic` with your corresponding ROS version name, if it
is not `melodic` (e.g. `kinetic`).

Open up a terminal, and enter:

```bash
source /opt/ros/melodic/setup.bash
mkdir -p ~/ros-vex-workspace/src; cd ~/ros-vex-workspace/src
git clone https://github.com/ros-drivers/rosserial.git
cd ..; catkin_make; catkin_make install
```

Next, generate a PROS project, which has the code that runs on the Cortex.
The project can exist anywhere (it does NOT need to be inside the ROS workspace).

```bash
source ~/ros-vex-workspace/install/setup.bash
rosrun rosserial_vex_cortex genscript.sh ~/path/to/prosproject
```

# Examples

These examples are made to run out-of-the-box, and they made to be proof-of-concepts of what ROS can provide.
ROS allows standardized messages to be sent to and from the Cortex and an outside computer, which allows for all kinds
of ideas and projects to be organized with messages!

To understand what is going on with the example code, look at the tutorials for the sister project, [Rosserial Arduino](http://wiki.ros.org/rosserial_arduino/Tutorials).
There are some differences between the two projects (namely, in the PROS c++ code, [global scope is not allowed](#limitations)).

Set up the physical download connection by plugging in 
the VEX Programming cable to the computer and the joystick, and then pluging the VEXnet keys into the Cortex and the joystick.
This connection serves as BOTH the downloading channel AND the serial connection, so leave the 
programming cable plugged in! (to alter the connection to use something else, see [physical serial connections](#physical-serial-connections)).
Between downloads, power-cycle the Joystick and Cortex for optimal usage.

## Hello World Example

```bash
source ~/your-workspace-name/install/setup.bash
cd ~/path/to/prosproject
pros make upload; roslaunch rosserial_vex_cortex hello_world.launch
```

If everything is working properly, you should see "hello world" messages in the terminal!

## Keyboard or Android Driving Example
This example showcases an integrated demo with a VEX EDR Robot, such as the [clawbot](https://www.vexrobotics.com/276-2600.html), and an alternative method of control: a keyboard!
Open up "src/twistdrive.cpp" and modify the motor control code, to specify how you want to control your robot's drive .
Modify `src/opcontrol.cpp` in your generated PROS project to include the `twistdrive.cpp` file instead of the `helloworld.cpp` file. Then, open a terminal and run the following:

```bash
source ~/your-workspace-name/install/setup.bash
cd /path/to/prosproject
pros make clean; pros make upload; roslaunch rosserial_vex_cortex minimal_robot.launch
```

Android phones can now control this robot.
Download this [ROS Map Navigation](https://play.google.com/store/apps/details?id=com.github.rosjava.android_apps.map_nav.kinetic) app.
Configure the app to connect to your computer, and you can now drive your robot with a phone!

For keyboard input instead, install the keyboard twist publisher: http://wiki.ros.org/teleop_twist_keyboard
Run the following in another terminal:

```bash
source /opt/ros/melodic/setup.bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
Now, you can use the keys listed in the command to drive the robot!

## Alternative Joystick Example
This is for using an alternative joystick, such as a Logitech Wireless Gamepad F710 or a PS3 controller.
This controlling example is extensibile: anything publishing a [sensor_msgs/Joy message](http://wiki.ros.org/sensor_msgs) can control your VEX robot, 
so feel free to program your own controller!
Open up `src/joydrive.cpp` and make modifications to make this demo work on your robot.
Modify `src/opcontrol.cpp` in your generated PROS project to include the `joydrive.cpp` file instead of the `helloworld.cpp` file. Then, open a terminal and run the following:

```bash
cd /path/to/prosproject
pros make upload
roslaunch rosserial_vex_cortex joystick.launch
```

Now, you can use the alternative joystick to control the robot!

# Physical Serial Connections
The optimal setup for this project is with two physical serial connections, one for rosserial to function, and one for debugging. The default connection for rosserial is the VEX Programming Cable, and the default debugging serial connection is UART2.

Since the The VEX programming cable provides the default rosserial connection, the baud rate of the ROS serial connecting node must be 115200 Hz, which is why the command reads: 

This overrides the default (57600) Hz. To switch the rosserial/debug serial connections, see `include/ros_lib/logger.h` in your generated PROS project.
If you do end up using UART1 or UART2 for rosserial instead of debugging,
open the `launch/` directory (which is inside the ROS workspace in `src/rosserial/rosserial_vex_cortex`).
Inside `launch/minimal_robot.launch`, update the `mainport` USB device argument to be `/dev/ttyUSB0`,
and update the `mainbaud` argument to be `57600`. 
There are also `debugport` and `debugbaud` arguments in the launchfile, which are unused for now but can be used for project generation later.

Also, the USB device path for the VEX Programming cable on Linux may either be `/dev/ttyACM0` or `/dev/ttyACM1`. To figure out which to use as an argument, use `pros lsusb`, or unplug/replug the cable from/into the computer and run `dmesg` and look at the last lines. Usually, unplugging the programming cable for 10 seconds and then re-plugging it should revert
the device back to `/dev/ttyUSB0`

Viewing the UART debug stream requires a [USB-serial adapter](https://www.adafruit.com/product/954) for your computer, and it needs to be plugged in correctly.
To set up the wires with the cable linked above, use this layout:
[layout](./uartdiagram.png)
You need male-male jumper wires to plug in this adapter.
Run `dmesg` right after plugging in the adapter to identify the USB device path - it is most likely `/dev/ttyUSB0`.

To view serial output from UART2, (instead of running `pros terminal`, which only works for the VEX Programming cable), use screen:
```bash
# to install screen: sudo apt-get install screen
screen /dev/ttyUSB0 57600
```
Again, you may need to change the above command to use the correct USB device path and baud rate, if you change the default configuration.

PROS does not provide a debugger, but it does provide printing.
use vexroslog(char* out, ...) just like you would use fprintf from the PROS API:

```bash
vexroslog("hello, my favorite number is %d", 3);
```
remember to include this header in your PROS code for this logging function!
```cpp
#include "logger.h"
```


# Generating Custom Messages
To design you own ROS messages, it is necessary to add the `msgs` directory to this project,
add the `message_generation` dependency, and make several modifications to this project's `CMakeLists.txt`.
This infrastructure is removed from this project by default,
because there are many useful built-in message types that are used commonly across ROS packages anyway.

See documentation and source of sister packages, such as `rosserial_arduino`, for more information about generating custom messages.

# Limitations

### Global Scope
Global scope variables causes segmentation faults. Unlike in the other rosserial examples
avoid putting objects/structs in the global scope.
Read the comments inside the PROS project files for clarification on where `global scope` is referring to.

To workaround not having global variables:
1. If possible, keep variables inside functions.
2. If an object/struct must be accessed across concurrently running tasks, use a global shared pointer or semaphore (see API.h for semaphore functions).

The second strategy should be used as a last-resort, because the better option is simply to use locally-scoped variables inside functions (e.g. declare variables in the body of `void opcontrol()`, and pass them to functions that need to use the variables).

This issue is most likely a side-effect of mixing C++ source, compiled with g++, and C source, compiled with C99. the initialization procedure for C++ static constructors is skipped for some reason (see [this issue](https://github.com/purduesigbots/pros/issues/48), although the fix provided does not work here).

### Platforms
This has been developed and tested on ROS melodic, but it should work on many earier/later versions as well.

# Speed
Over the VEX Programming cable/VEXNet wireless connection, the simplest messages can stream at upwards of 200Hz. More complex messages, such as sensor_msgs::JointState, can be published at 50hz. The baud rate degrades as the distance from the cortex and the Joystick increases, however.

A wired UART1 or UART2 connection should have higher stability and frequency with arbitrarily-sized messages, with no degradation in baud rate as distance increases.

# Troubleshooting
If your program crashes, it may be difficult to debug effectively without a second USB-serial connection for debugging messages, since the crash message will only print to stdout. run `pros terminal` to view whether or not the program is crashing (this only works if you can switch rosserial to use a UART connection - see Physical Serial Connections). Make sure to test the individual pieces of your program seperately to ensure they work properly, before integrating them with the program as a whole. Make sure to power cycle the Cortex if it crashes.

This project is similar to [Rosserial Arduino](http://wiki.ros.org/rosserial_arduino/Tutorials) in usage, so refer to these tutorials for even more information.

