# rosserial for VEX Cortex

This package contains everything needed to run rosserial on the [VEX Cortex](https://www.vexrobotics.com/276-2194.html), on the [PROS Kernel](https://pros.cs.purdue.edu/cortex/index.html).

# Requirements
Software:
1. Linux (Only tested on Ubuntu 18.04LTS) (possible, but not tested, on windows + virtual machines with USB support)
2. ROS installed on Linux (Only tested on ROS Melodic) - [installation guide](http://wiki.ros.org/melodic/Installation/Source).
3. PROS installed on Linux - [installation guide](https://pros.cs.purdue.edu/cortex/getting-started/index.html)
Hardware:
1. VEX essentials: VEX Cortex, Joystick, VexNet keys, battery
2. VEX Programming Cable
3. (recommended, for debugging) a [USB-serial adapter](https://www.adafruit.com/product/954) 
4. Three Male-Male jumper wires for USB-serial adapter

# Setup
Setup a ROS workspace and build rosserial packages (including rosserial_vex_cortex) from source:
```bash
source /opt/ros/melodic/setup.bash # or replace melodic with your corresponding ROS version name
mkdir -p <your-workspace-name>/src
cd <your-workspace-name>/src
git clone https://github.com/ros-drivers/rosserial.git
cd ..
catkin_make
catkin_make install # this will generate folders in the workspace that contain executable scripts. 
```

# Usage Example (Linux)
This will show you how to run the "hello world" example PROS project. Plug in the VEX Programming cable to the computer and the joystick, plug the VexNet keys into the Cortex and the joystick. Power cycle the Joystick and Cortex between cortex downloads for optimal usage.

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
see the `Serial Connections` section below, for more information about how the serial ports are used.

### Step 4: Display a topic in a terminal (technically optional)
open another terminal window and run:
```bash
source /opt/ros/melodic/setup.bash # or replace melodic with your corresponding ROS version name
rostopic echo chatter
```
this will display any messages that come through on the "chatter" topic.

If everyting is working properly, then the terminal from step 2 should show outputs of `"Hello World!"` This means the bridge between the cortex and ROS is established.
You are now able to use ROS with the VEX Cortex!


# Physical Serial Connections
The optimal setup for this project is with two physical serial connections, one for rosserial to function, and one for debugging. The default connection for rosserial is the VEX Programming Cable, and the default debugging serial connection is UART2.

Since the The VEX programming cable provides the default rosserial connection, the baud rate of the ROS serial connecting node must be 115200 Hz, which is why the command reads: 

```bash
rosserial_arduino serial_node _port:=/dev/ttyACM0 _baud:=115200
```
This overrides the default (57600) Hz.To switch the rosserial/debug serial connections, see `logger.h` in your generated PROS project. If you do end up using UART1 or UART2 for rosserial instead of debugging, update the USB device argument: `_port:=/dev/ttyUSB0` instead, and update the baud rate argument, or simply remove it for the default 57600 Hz.

Also, the USB device path for the VEX Programming cable on linux may either be `/dev/ttyACM0` or `/dev/ttyACM1`. To figure out which to use as an argument, use `pros lsusb`, or unplug/replug the cable from/into the computer and run `dmesg` and look at the last lines.

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
remember to include this header in your code for logging!
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
Global scope variables causes segmentation faults. Unlike in the other rosserial examples, avoid putting objects/structs in the global scope. Read the comments inside the templates for clarification on where `global scope` is referring to.

To workaround not having global variables:
1. If possible, keep variables inside functions.
2. If an object/struct must be accessed across concurrently running tasks, use a global shared pointer or semaphore (see API.h for semaphore functions).

The second strategy should be used as a last-resort, because the better option is simply to use locally-scoped variables inside functions (e.g. declare variables in the body of `void opcontrol()`, and pass them to functions that need to use the variables).

This issue is most likely a side-effect of mixing C++ source, compiled with g++, and C source, compiled with C99. the initialization procedure for C++ static constructors is skipped for some reason (see [this issue](https://github.com/purduesigbots/pros/issues/48), although the fix provided does not work here).

### Platforms
This has been developed and tested on ROS melodic, but it should work on many earier/later versions as well.

# Speed
Over the VEX Programming cable/VexNet wireless connection, the simplest messages can stream at upwards of 200Hz. More complex messages, such as sensor_msgs::JointState, can be published at 50hz. The baud rate degrades as the distance from the cortex and the Joystick increases, however.

A wired UART1 or UART2 connection should have higher stability and frequency with arbitrarily-sized messages, with no degradation in baud rate as distance increases.

# Troubleshooting
If your program crashes, it may be difficult to debug effectively without a second USB-serial connection for debugging messages, since the crash message will only print to stdout. run `pros terminal` to view whether or not the program is crashing (this only works if you can switch rosserial to use a UART connection - see Physical Serial Connections). Make sure to test the individual pieces of your program seperately to ensure they work properly, before integrating them with the program as a whole. Make sure to power cycle the Cortex if it crashes.


