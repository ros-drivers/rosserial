This package contains Arduino-specific extensions required to run
[rosserial_client](https://index.ros.org/p/rosserial_client) on an
Arduino. It is meant to demonstrate how easy it is to integrate custom
hardware and cheap sensors into your ROS project using an Arduino. The
Tutorials of this package will walk you through setting up your Arduino
environment, creating a few example sketches and explain where to
purchase the additional hardware.

## Special defines

Some features can be enabled depending of `#define` statements added
before including `ros.h` in the sketch.

  -- ----------------------------------------------------------------- -- -------------------------------------------- -- ------------------------- --
     **Platform**                                                         **Feature**                                     **`#define`**             
     Arduino based on 32u4 (Leonardo, Micro)                              USB Serial (not pin 0 & 1)                      `USE_USBCON`              
     All Arduino variants                                                 Use Ethernet shield for rosserial over TCP      `ROSSERIAL_ARDUINO_TCP`   
     Teensy 3.x                                                           Hardware Serial (not USB)                       `USE_TEENSY_HW_SERIAL`    
     [STM32F1](https://github.com/rogerclarkmelbourne/Arduino_STM32)      Hardware Serial (not USB)                       `USE_STM32_HW_SERIAL`     
     [STM32Duino](https://github.com/stm32duino/Arduino_Core_STM32)       Use native Ethernet for rosserial over TCP      `STM32ETHERNET`           
  -- ----------------------------------------------------------------- -- -------------------------------------------- -- ------------------------- --



```wiki
'''STM32ETHERNET PR not yet merged'''

https://github.com/ros-drivers/rosserial/pull/395
```



### Example

For example, to use the `ROSSERIAL_ARDUINO_TCP` define, see the
[TcpHelloWorld](https://github.com/ros-drivers/rosserial/blob/dd76994c67c5e4997ef64837c07afb4eb0d4df27/rosserial_arduino/src/ros_lib/examples/TcpHelloWorld/TcpHelloWorld.ino#L15)
example.

## Tutorials

See also [rosserial/Overview](https://index.ros.org/p/rosserial).

- [Arduino IDE Setup](#arduino-ide-setup):
  This tutorial shows step-by-step how to setup up the Arduino IDE to
  use rosserial.
- [Hello World (example publisher)](#hello-world-example-publisher):
  This tutorial shows step by step how to create a publisher using
  rosserial.
- [Blink (example subscriber)](#blink-example-subscriber):
  This tutorial shows step by step how to use rosserial with
  subscribers.
- [Logging in rosserial](#logging-in-rosserial):
  This tutorial shows step-by-step how to use logging for rosserial.
- [Using Time and TF](#using-time-and-tf):
  This tutorial shows how to use ros::Time and TF to create a tf
  publisher on the Arduino.
- [Adding Custom Messages (Generating Message Header File)](#adding-custom-messages-generating-message-header-file):
  This tutorial shows how to generate message header files for using new
  message packages with rosserial (i.e. how to add custom message type
  to your application with rosserial).
- [Temperature Sensor](#temperature-sensor):
  Measuring Temperature using the TMP102
- [Push Button](#push-button):
  Monitor a push button and publish its state in ROS
- [CMake with rosserial_arduino](#cmake-with-rosserial_arduino):
  This tutorial shows how to use the CMake build system with
  rosserial_arduino.
- [Servo Controller Example](#servo-controller-example):
  Tutorial for controlling an R/C servo with rosserial and an Arduino
- [IR Ranger Tutorial](#ir-ranger-tutorial):
  Using an IR Ranger with rosserial and an Arduino
- [SRF08 Ultrasonic Ranger](#srf08-ultrasonic-ranger):
  In this tutorial, we will use an Arduino and a
  [SRF08 Ultrasonic Ranger](http://www.acroname.com/robotics/parts/R145-SRF08.html)
  as a Range Finder.The SRF08 communicates with an Arduino over SPI/I2C.
- [BlinkM Tutorial](#blinkm-tutorial):
  Control a BlinkM through ROS using an Arduino and rosserial
- [Arduino Oscilloscope](#arduino-oscilloscope):
  In this tutorial, we will be making a poor man's oscilloscope using
  the Arduino's ADC (analog to digital converter) and rqt_plot.
- [Advanced Configuration for NodeHandle and ArduinoHardware.](#advanced-configuration-for-nodehandle-and-arduinohardware):
  This tutorial shows step by step how to configure NodeHandle and
  ArduinoHareware to better suit user needs.

## Arduino IDE Setup

- **Note:** This tutorial assumes that you have completed the previous
  tutorials: [ROS Tutorials](ROS/Tutorials).
- **Description:** This tutorial shows step-by-step how to setup up the
  Arduino IDE to use rosserial.
- **Tutorial Level:** BEGINNER
- **Next Tutorial:**
  [Hello World (example publisher)](#hello-world-example-publisher)

### Introduction

The Arduino and Arduino IDE are great tools for quickly and easily
programming hardware. Using the
rosserial_arduino package, you can use
ROS directly with the Arduino IDE.
[rosserial](https://index.ros.org/p/rosserial) provides a ROS
communication protocol that works over your Arduino's UART. It allows
your Arduino to be a full fledged ROS node which can directly publish
and subscribe to ROS messages, publish TF transforms, and get the ROS
system time.

NOTE: If you do not already have an Arduino IDE installed, download it
[from the Arduino website](http://arduino.cc/en/Main/Software). It is
best to install the application into a folder on the application PATH,
the desktop, or home folder. Once installed, launch the application to
select your sketchbook location. (See
[arduino official website](http://www.arduino.cc/en/Guide/Environment),
sketchbook is a standard place to store your programs, or sketches).
Close the IDE when done.

Our ROS bindings are implemented as an Arduino library. Like all Arduino
libraries, `ros_lib` works by putting its library implementation into
the *libraries* folder of your sketchbook. If there is not already a
*libraries* folder in your sketchbook, make one. You can then install
the library using the instructions below.

In order to use the rosserial libraries in your own code, you must first
put



```
#include <ros.h>
```

 prior to including any other header files, e.g.



```
#include <std_msgs/String.h>
```

 otherwise the Arduino IDE will not be able to
locate them.

### Installing the Software

#### Installing on the ROS workstation

You have 2 options of how to install related libraries.

##### (RECOMMENDED) Installing Binaries on the ROS workstation

You can install rosserial for Arduino by running:



```
sudo apt-get install ros-${ROS_DISTRO}-rosserial-arduino
sudo apt-get install ros-${ROS_DISTRO}-rosserial
```



##### Installing from Source onto the ROS workstation

Source build instructions are different for groovy+ (catkin) than for
earlier (rosbuild) releases. Select the build system based on your
release to see appropriate instructions.



```
{#!wiki buildsystem catkin

Rosserial has been catkin-ized since the groovy release, and the workflow is a bit different from fuerte and earlier releases. Rather than running the library generator over each package you want to use, you run it once and generate libraries for all installed messages. In the instructions below, <ws> represents your catkin workspace.

{{{
  cd <ws>/src
  git clone https://github.com/ros-drivers/rosserial.git
  cd <ws>
  catkin_make
  catkin_make install
```



These commands clone rosserial from the github repository, generate the
rosserial_msgs needed for communication, and make the library files in
the <ws>/devel/lib directory.

**Note:** currently you HAVE to run catkin_make install, otherwise
portions of the ros_lib directory will be missing. This will hopefully
be fixed soon. }}}} 

```
{#!wiki buildsystem rosbuild

{{{
hg clone https://kforge.ros.org/rosserial/hg rosserial
rosmake rosserial_arduino
```

 These commands clone
rosserial from the kforge repository using mercurial, generate the
rosserial_msgs needed for communication, and make the `ros_lib`
library. }}}}

#### Install ros_lib into the Arduino Environment

The preceding installation steps created the necessary libraries, now
the following will create the ros_lib folder that the Arduino build
environment needs to enable Arduino programs to interact with ROS.

In the steps below, <sketchbook> is the directory where the Linux
Arduino environment saves your sketches. Typically this is a directory
called *sketchbook* or *Arduino* in your home directory. e.g cd
/Arduino/libraries

Alternately, you can install into a Windows Arduino environment.

Ros_lib installation instructions are different for groovy source
(catkin) than for earlier (rosbuild) or binary releases. Be sure you've
selected the correct build system above to see appropriate
instructions - catkin for a groovy source build, rosbuild otherwise.



```
{#!wiki buildsystem catkin

'''Note:''' you have to delete libraries/ros_lib, if present, in order to regenerate as its existence causes an error.
"rosrun rosserial_arduino make_libraries.py" creates the ros_lib directory.

{{{
  cd <sketchbook>/libraries
  rm -rf ros_lib
  rosrun rosserial_arduino make_libraries.py .
```



If you are building the Arduino on Windows, you need to create the
ros_lib folder in some convenient directory.



```
cd <some_empty_directory>
  rosrun rosserial_arduino make_libraries.py .
```

 }}}} 

```
{#!wiki buildsystem rosbuild
Now that you've installed either from source or debs, all you have to do is copy the `rosserial_arduino/libraries` directory into your Arduino sketchbook:
{{{
roscd rosserial_arduino/src
cp -r ros_lib <sketchbook>/libraries/ros_lib
```

 }}}} If you are
building Arduino on Windows, copy the ros_lib directory from Linux to
the Windows sytem's sketchbook/libraries folder (typically found in My
Documents).

- Note: Currently you can install the Arduino libaries directly in the
  Arduino IDE. Just open the Library Manager from the IDE menu in Sketch
  -> Include Library -> Manage Library. Then search for "rosserial".
  This is useful if you need to work on an Arduino sketch but don't
  want to setup a full ROS workstation.

### Finishing Up

After restarting your IDE, you should see ros_lib listed under examples:

![align="middle"](doc/arduino_ide_examples_screenshot.png)

And under Windows:

![align="middle"](doc/arduino_ide_examples_screenshot_windows.png)

## Hello World (example publisher)

- **Note:** This tutorial assumes that you have completed the previous
  tutorials:
  [Arduino IDE Setup](#arduino-ide-setup).
- **Description:** This tutorial shows step by step how to create a
  publisher using rosserial.
- **Tutorial Level:** BEGINNER
- **Next Tutorial:**
  [Blink (example subscriber)](#blink-example-subscriber)

### Hello World: Creating a Publisher

#### The Code

We'll start our exploration into rosserial by creating a "hello
world" program for our Arduino. (Note: the Arduino community often
calls source code for programs a "sketch", we will use the same
convention below). If you have followed the
[Arduino IDE Setup](#arduino-ide-setup)
tutorial, you'll be able to open the sketch below by choosing `ros_lib
-> HelloWorld` from the Arduino `examples` menu.

This should open the following code in your IDE:



```cplusplus
/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

// Use the following line if you have a Leonardo or MKR1000
//#define USE_USBCON

#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

void setup()
{
  nh.initNode();
  nh.advertise(chatter);
}

void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
}
```



#### The Code Explained

Now, let's break the code down.



```
// Use the following line if you have a Leonardo or MKR1000
```

 Use the following line if you have a Leonardo
or MKR1000 or any other device that has a native USB port, which is
different than on the Arduino UNO, MEGA. Otherwise you will receive an
error similar to the following:



```
[ERROR] [1524089506.982801]: Unable to sync with device; possible link problem or link software version mismatch such as hydro rosserial_python with groovy Arduino
```

 

```
#include <ros.h>
#include <std_msgs/String.h>
```

 As a part of every ROS
Arduino program, you need to include the `ros.h` header file and
header files for any messages that you will be using.



```
ros::NodeHandle nh;
```

 Next, we need to instantiate the node handle,
which allows our program to create publishers and subscribers. The node
handle also takes care of serial port communications.



```
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";
```

 We need to instantiate the publishers and
subscribers that we will be using. Here we instantiate a Publisher with
a topic name of "chatter". The second parameter to Publisher is a
reference to the message instance to be used for publishing.



```
void setup()
{
  nh.initNode();
  nh.advertise(chatter);
}
```

 In the Arduino setup function you then need to
initialize your ROS node handle, advertise any topics being published,
and subscribe to any topics you wish to listen to.



```
void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
}
```

 Finally, in the loop function, the node
publishes "Hello World" and calls `ros::spinOnce()` where all of the
ROS communication callbacks are handled.

#### Compiling the Code

To compile the code, use the compile function within the Arduino IDE.
This is no different from uploading any other sketch. Once the
compilation is completed, you will receive a message about program
storage space and dynamic memory usage, similar to this:



```
Sketch uses 9,392 bytes (29%) of program storage space. Maximum is 32,256 bytes.
Global variables use 1,356 bytes (66%) of dynamic memory, leaving 692 bytes for local variables. Maximum is 2,048 bytes.
```

 The program storage space is less important, as
long as you use less than the total storage available, your code will
run fine.

The dynamic memory usage is something you should be careful about, as
local variables inside of an function, unallocated objects, interrupts,
and stacks all can change how much free memory you have. If you run out
of the memory, your microcontroller will simply freeze with no debug
information.

If you encounters random freezes or your Arduino unable to communicate
with ROS while your code compiles fine, but dynamic memory usage is
high, you might need to reduce the memory used by `rosserial`. Refer
to [rosserial Limitations](http://wiki.ros.org/rosserial?#Limitations)
and
[NodeHandle and ArduinoHardware](#advanced-configuration-for-nodehandle-and-arduinohardware)
for more information.

#### Uploading the Code

To upload the code to your Arduino, use the upload function within the
Arduino IDE. This is no different from uploading any other sketch.

#### Running the Code

Now, launch the [roscore](roscore) in a new terminal window:



```
roscore
```

 Next, run the rosserial client application that
forwards your Arduino messages to the rest of ROS. Make sure to use the
correct serial port:



```
rosrun rosserial_python serial_node.py /dev/ttyUSB0
```

 { }}}} { }}}} Alternatively, if you want the
ability to programmatically reset your Arduino, run using:



```
rosrun rosserial_arduino serial_node.py _port:=/dev/ttyUSB0
```

 This will automatically provide a service
endpoint at `reset_arduino` that you can call which will have the same
effect as pressing the Arduino's reset button.

Finally, watch the greetings come in from your Arduino by launching a
new terminal window and entering:



```
rostopic echo chatter
```



### Further Reading

Please see [rosserial/Overview](https://index.ros.org/p/rosserial) for
more information on publishers and subscribers. Also see
[limitations](https://index.ros.org/p/rosserial) for information about
more complex data types.

To generate header files for your custom messages, check
[rosserial_arduino/Tutorials/Adding Custom Messages](http://wiki.ros.org/rosserial_arduino/Tutorials/Adding%20Custom%20Messages)
tutorial.

## Blink (example subscriber)

- **Note:** This tutorial assumes that you have completed the previous
  tutorials:
  [Hello World (example Publisher)](#hello-world-example-publisher).
- **Description:** This tutorial shows step by step how to use rosserial
  with subscribers.
- **Tutorial Level:** BEGINNER
- **Next Tutorial:**
  [Logging](#logging-in-rosserial)

### Blink: Creating a Subscriber

#### The Code

Now that we've created a ROS publisher in the
[previous tutorial](#hello-world-example-publisher),
we'll create a subscriber. If you have followed the
[Arduino IDE Setup tutorial](#arduino-ide-setup),
you'll be able to open the sketch below by choosing `ros_lib ->
Blink` from the Arduino examples menu.

This should open the following code in your IDE:



```cplusplus
/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/Empty.h>

ros::NodeHandle nh;

void messageCb( const std_msgs::Empty& toggle_msg){
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
}

ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );

void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
```



#### The Code Explained

Now, let's break the code down.



```
#include <ros.h>
#include <std_msgs/Empty.h>
```



As before, we need to include the `ros.h` as with any other ROS
Arduino program. We also include the header files for messages, in this
case, the `Empty` message.



```
ros::NodeHandle nh;
```



Next, we need to instantiate the node handle, which allows our program
to create publishers and subscribers. The node handle also takes care of
serial port communications.



```
void messageCb( const std_msgs::Empty& toggle_msg){
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
}
```



We then create the callback function for our subscriber. The call back
function must take a constant reference of a message as its argument. In
our callback `messageCb`, the type of message is `std_msgs::Empty`
and the message name will be `toggle_msg`.

Inside our callback, we could reference `toggle_msg`, but since it is
empty, there is no need to. We just blink the LED on the Arduino every
time we receive a message.



```
ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );
```



We need to instantiate the publishers and subscribers that we will be
using. Here we instantiate a Subscriber with a topic name of
"toggle_led" and type `std_msgs::Empty`. With Subscribers, you must
remember to template the subscriber upon the message. Its two arguments
are the topic it will be subscribing to and the callback function it
will be using.



```
void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}
```



In the Arduino setup function you then need to initialize your ROS node
handle, advertise any topics being published, and subscribe to any
topics you wish to listen to.



```
void loop()
{
  nh.spinOnce();
  delay(1);
}
```



Finally, in the loop function we call ros::spinOnce() where all of the
ROS communication callbacks are handled. We don't need to do any
additional processing in the `loop()`, since ros::spinOnce() will
handle passing messages to the subscriber callback.

#### Uploading the Code

To upload the code to your Arduino, use the upload function within the
Arduino IDE. This is no different from uploading any other sketch.

#### Running the Code

Now, launch the [roscore](roscore) in a new terminal window:



```
roscore
```

 Next, run the rosserial client application that
forwards your Arduino messages to the rest of ROS. Make sure to use the
correct serial port:



```
rosrun rosserial_python serial_node.py /dev/ttyUSB0
```

 Finally, you can toggle the LED using
[rostopic](https://index.ros.org/p/rostopic):



```
rostopic pub toggle_led std_msgs/Empty --once
```



### Further Reading

Please see [rosserial/Overview](https://index.ros.org/p/rosserial) for
more information on publishers and subscribers. Also see
[limitations](https://index.ros.org/p/rosserial) for information about
more complex data types.

To generate header files for your custom messages, check
[rosserial_arduino/Tutorials/Adding Custom Messages](http://wiki.ros.org/rosserial_arduino/Tutorials/Adding%20Custom%20Messages)
tutorial.

## Logging in rosserial

- **Note:** This tutorial assumes that you have completed the previous
  tutorials:
  [Hello World](#hello-world-example-publisher).
- **Description:** This tutorial shows step-by-step how to use logging
  for rosserial.
- **Tutorial Level:** BEGINNER
- **Next Tutorial:**
  [Using Time and TF](#using-time-and-tf)

### Logging

Like any ROS node, you can log messages at the appropriate verbosity
level. [rosserial](https://index.ros.org/p/rosserial) forwards the
string to the standard ROS network and outputs to rosout and to its log
file.

If you try to log information before the device has connected to ROS,
the logged information will be lost.

Keep in mind that for low memory systems, this kind of logging is very
expensive. You need to store the string in memory before you send it.
For more memory thrifty logging, try defining a custom message type with
exactly the information you need sent over. You can then view your
program's status by observing it using rostopic echo.

#### The Code



```cplusplus
#include <ros.h>
ros::NodeHandle nh;

void setup()
{
  nh.initNode();
}

void loop()
{
  //wait until you are actually connected
  while (!nh.connected())
  {
    nh.spinOnce();
  }
 
  //Now you can send all of the statements you want
  //to be logged at the appropriate verbosity level
  nh.logdebug("Debug Statement");
  nh.loginfo("Program info");
  nh.logwarn("Warnings.");
  nh.logerror("Errors..");
  nh.logfatal("Fatalities!");
  delay(5000);
}
```



#### Uploading the Code

To upload the code to your Arduino, use the upload function within the
Arduino IDE. This is no different from uploading any other sketch.

#### Running the Code

Now, launch the [roscore](roscore) in a new terminal window:


```
roscore
```



Next, run the rosserial client application that forwards your Arduino
messages to the rest of ROS. Make sure to use the correct serial port:



```
rosrun rosserial_python serial_node.py /dev/ttyUSB0
```





```

```

}}} 

```

```

}}}

You should be able to see all the log messages in the same terminal that
you used to run the serial node.

### Further Reading

Please see [rosserial/Overview](https://index.ros.org/p/rosserial) for
more information on publishers and subscribers. Also see
[limitations](https://index.ros.org/p/rosserial) for information about
more complex data types.

## Using Time and TF

- **Description:** This tutorial shows how to use ros::Time and TF to
  create a tf publisher on the Arduino.
- **Tutorial Level:** INTERMEDIATE
- **Next Tutorial:**
  [Adding Custom Messages](#adding-custom-messages-generating-message-header-file)

### Using Time and TF on the Arduino

The `rosserial_arduino` package contains libraries for generating
timestamps on the Arduino which are synchronized with the PC/Tablet on
which the roscore instance is running. This tutorial shows how to access
time, using an example of publishing a `tf` transform.

#### The Code

If you have followed the Arduino IDE Setup tutorial, you'll be able to
open the sketch below by choosing `ros_lib -> TimeTf` from the
Arduino examples menu.

This should open the following code in your IDE:



```cplusplus
/*
 * rosserial Time and TF Example
 * Publishes a transform at current time
 */

#include <ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>

ros::NodeHandle  nh;

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

char base_link[] = "/base_link";
char odom[] = "/odom";

void setup()
{
  nh.initNode();
  broadcaster.init(nh);
}

void loop()
{
  t.header.frame_id = odom;
  t.child_frame_id = base_link;
  t.transform.translation.x = 1.0;
  t.transform.rotation.x = 0.0;
  t.transform.rotation.y = 0.0;
  t.transform.rotation.z = 0.0;
  t.transform.rotation.w = 1.0;
  t.header.stamp = nh.now();
  broadcaster.sendTransform(t);
  nh.spinOnce();
  delay(10);
}
```



#### The Code Explained

Now, let's break the code down. We need to include our typical ROS
stuff, as well as the transform broadcaster. Next, we instantiate a
`TransformStamped` message to use, and a broadcaster. We also need to
specify the names of the frames we are publishing a transform for.
Inside the `setup()` function, we have to call `init()` on the
!TransformBroadcaster with the node handle as a parameter. Without doing
this, the broadcaster will not publish correctly. Inside the `loop()`
function, we fill in the fields of our transform. The frame IDs are set
to the correct string names, and the values of the translation and
rotation are set. Calling `nh.now()` returns the current time, as a
`ros::Time` instance, just like when using
[roscpp's ros::Time::now()](https://index.ros.org/p/roscpp). Finally,
we publish the transform, and then `spinOnce` and wait a bit before
doing it again. Transforms should always be published at a regular rate,
although usually the data fields will be filled in from real data.

#### Uploading the Code

To upload the code to your Arduino, use the upload function within the
Arduino IDE. This is no different from uploading any other sketch.

#### Running the Code

Now, launch the [roscore](roscore) in a new terminal window:



```
roscore
```

 Next, run the rosserial client application that
forwards your Arduino messages to the rest of ROS. Make sure to use the
correct serial port:

{ }}}} { }}}} { }}}} Finally, you can check the transform using:



```
rosrun tf tf_echo odom base_link
```

 This by default will echo the transform being
sent from the Arduino approximately once per second. Note that the
Arduino is likely broadcasting at a higher rate (Mega2560 publishes at
about 57hz when connected at 57600)

Or, by running:



```
rosrun tf view_frames
```

 This last command will run for about 5 seconds
and generate a 'frames.pdf' file in the directory you ran the command
in.

### Further Reading

Please see [rosserial/Overview](https://index.ros.org/p/rosserial) for
more information using Time instances.

## Adding Custom Messages (Generating Message Header File)

- **Description:** This tutorial shows how to generate message header
  files for using new message packages with rosserial (i.e. how to add
  custom message type to your application with rosserial).
- **Tutorial Level:** INTERMEDIATE
- **Next Tutorial:**
  [Measuring Temperature](#temperature-sensor)

### Overview

This tutorial contains the same information as
[Generating Message Header File](https://index.ros.org/p/rosserial_client).
Message header files are generated for many important message types when
you install [rosserial](https://index.ros.org/p/rosserial). If you need
to use other message types, or you have your own custom message types,
you will need to generate the appropriate headers.

### Adding Custom Messages(Generating Message Header File) == In groovy and above, you no longer need to run the make_library script for each package, instead, it is a one-time thing that generates message headers for all packages on your path. The script is also specific to the platform you are using (for instance, rosserial_arduino or rosserial_embedded_linux). The make_library has moved from rosserial_arduino to rosserial_client.

The [rosserial_client](https://index.ros.org/p/rosserial_client) package
includes a tool for generating the required header files from
[message definition files](Messages).



```
rosrun rosserial_client make_library.py path_to_libraries your_message_package
```

 For instance, suppose I need to generate
headers for message definitions contained in my `crazy_msgs` package,
and *ros_lib* is located at '/sketchbook/libraries/ros_lib':



```
rosrun rosserial_client make_library.py ~/sketchbook/libraries crazy_msgs
```

 This command will create the headers, and
locate them in `/sketchbook/libraries/ros_lib/crazy_msgs/`.

## Temperature Sensor

- **Description:** Measuring Temperature using the TMP102
- **Tutorial Level:** BEGINNER
- **Next Tutorial:**
  [Push Button](#push-button)

In this tutorial, we will use an Arduino and a
[TMP102 temperature sensor](http://focus.ti.com/docs/prod/folders/print/tmp102.html)
to measure the ambient temperature in a room. The tutorial's goals are
two fold. The first is to show an example temperature sensing node. The
second is to demonstrate the Arduino as an I2C interface for ROS. The
TMP102 is one of many pieces of hardware that use I2C to communicate.
You can now interface SPI/I2C to ROS with ease using an Arduino.

### Hardware

For this tutorial, you will need an Arduino and a
[TMP102 Breakout board from Sparkfun](http://www.sparkfun.com/products/9418).
This board is a great little temperature sensor that can measure
temperature with a 0.625 degree C resolution. Connecting it to an
Arduino is very straightforward. It is a 3.3V sensor communication over
I2C so it connects to the Arduino's 3.3V output, GND, SDA, SCL pins.
SDA and SCL are the I2C data line and I2C clock line respectively. They
are exposed on the Arduino board as analog pins 4 and 5.

![width="500 height=300"](doc/tmp102.png)

Diagram from [here](http://bildr.org/2011/01/tmp102-arduino)

### Code



```cplusplus

#include <Wire.h>
#include <ros.h>
#include <std_msgs/Float32.h>


//Set up the ros node and publisher
std_msgs::Float32 temp_msg;
ros::Publisher pub_temp("temperature", &temp_msg);
ros::NodeHandle nh;

int sensorAddress = 0x91 >> 1;  // From datasheet sensor address is 0x91
                                // shift the address 1 bit right,
                                 //the Wire library only needs the 7
                                // most significant bits for the address

void setup()
{
  Wire.begin();        // join i2c bus (address optional for master)

  nh.initNode();
  nh.advertise(pub_temp);

}

long publisher_timer;

void loop()
{

  if (millis() > publisher_timer) {
  // step 1: request reading from sensor
    Wire.requestFrom(sensorAddress,2);
    delay(10);
    if (2 <= Wire.available())  // if two bytes were received
    {
      byte msb;
      byte lsb;
      int temperature;

      msb = Wire.read();  // receive high byte (full degrees)
      lsb = Wire.read();  // receive low byte (fraction degrees)
      temperature = ((msb) << 4);  // MSB
      temperature |= (lsb >> 4);   // LSB

      temp_msg.data = temperature*0.0625;
      pub_temp.publish(&temp_msg);
    }

  publisher_timer = millis() + 1000; //publish once a second
  }

  nh.spinOnce();
}
```

 The special bit of code in this example is the
use of Arduino's
[Wire library](http://www.arduino.cc/en/Reference/Wire). Wire is a I2C
library that simplifies reading and writing to the I2C bus.

### Testing



```
roscore
```





```

```

}}} 

```

```

}}}



```
rostopic echo temperature
```

## Push Button

- **Description:** Monitor a push button and publish its state in ROS
- **Tutorial Level:** BEGINNER
- **Next Tutorial:**
  [CMake](#cmake-with-rosserial_arduino)

This tutorial describes one of the simplest and most common pieces of
hardware you might want to integrate into your ROS system: a push
button. A button and be an input device to command your robot to do its
next task, a sensor to detect of a box is opened, or an important safety
feature to stop a motor from moving past a joints limits.

In this tutorial, we will write a rosserial_arduino node that will
monitor the state of a normally off button on pin 7. It will publish a
boolean message to the topic "pushed" everytime the button's state
changes.

### Hardware

For this tutorial, you need an
[Arduino](http://www.sparkfun.com/products/9950) and some sort of
normally off momentary switch or push button. This
[microswitch from Sparkfun](http://www.sparkfun.com/products/9414) is a
good choice.

![attachment:button_hardware.png](doc/button_hardware.png)

For those of you who are used to hardware design, you will notice that
there is no pull-up resistor on switch input. This is because the
Arduino has built in pullup resistors. Pin 7 is pulled high until the
button is pressed and connected to ground.

### Code

The code for this program is below. To use the code, copy it directly
into a fresh sketch in the Arduino IDE. 

```cplusplus
/* 
 * Button Example for Rosserial
 */

#include <ros.h>
#include <std_msgs/Bool.h>


ros::NodeHandle nh;

std_msgs::Bool pushed_msg;
ros::Publisher pub_button("pushed", &pushed_msg);

const int button_pin = 7;
const int led_pin = 13;

bool last_reading;
long last_debounce_time=0;
long debounce_delay=50;
bool published = true;

void setup()
{
  nh.initNode();
  nh.advertise(pub_button);
  
  //initialize an LED output pin 
  //and a input pin for our push button
  pinMode(led_pin, OUTPUT);
  pinMode(button_pin, INPUT);
  
  //Enable the pullup resistor on the button
  digitalWrite(button_pin, HIGH);
  
  //The button is a normally button
  last_reading = ! digitalRead(button_pin);
 
}

void loop()
{
  
  bool reading = ! digitalRead(button_pin);
  
  if (last_reading!= reading){
      last_debounce_time = millis();
      published = false;
  }
  
  //if the button value has not changed during the debounce delay
  // we know it is stable
  if ( !published && (millis() - last_debounce_time)  > debounce_delay) {
    digitalWrite(led_pin, reading);
    pushed_msg.data = reading;
    pub_button.publish(&pushed_msg);
    published = true;
  }

  last_reading = reading;
  
  nh.spinOnce();
}
```



This code is a typical example of how rosserial_arduino is used. The
rosserial objects are globally declared.

Initialized in the setup() function

and then publishing is performed in the "void loop()" when the
button's state changes.

#### Monitoring the Button

This sensor should not spam messages about button's state but should
publish an update only when the button is pressed or released. To
monitor this change we need to remember the button's last state and
know how long this state has been valid. Then once the state has
changed, the sensor should debounce the button. It should wait "long
enough" so we know that the button value has settled and is not
oscillating around. This is because the voltage at the input pin will
bounce around (as shown in the diagram below) as the mechanical contacts
bounce back and forth during contact and release.

![attachment:debounce.png](doc/debounce.png)

In the code, this debouncing is performed is a few places. First, there
are global variables storing the button's last value, the time that
reading was taken, the delay period for the debouncing, and if it was
published.

Next, in the void setup(), we initialize the arduino pins for io and
pullup resistors. We also initialize our state variables.

Finally, in void loop(), the code reads the button at every loop, checks
to see if the button state has changed, and then checks to see if the
button state is stable and has been published.

### Testing

Startup roscore 

```
roscore
```



In a new terminal window, run the rosserial_python serial_node.py. Make
sure to use the correct serial port. 

```

```

}}}


```

```

}}}

Now watch the button's value on the pushed topic


```
rostopic echo pushed
```

## CMake with rosserial_arduino

- **Note:** This tutorial assumes that you have completed the previous
  tutorials:
  [Hello World Publisher](#hello-world-example-publisher).
- **Description:** This tutorial shows how to use the CMake build system
  with rosserial_arduino.
- **Tutorial Level:** ADVANCED
- **Next Tutorial:**
  [Servo Controller](#servo-controller-example)

When you are doing large software projects, the Arduino IDE quickly
becomes unwieldy. You often want to be able to compile your project from
the command line or use a different IDE like Eclipse where you can use
autocompletion. Finally, by using rosserial_client CMake infrastructure,
you can build and distribute firmwares using the ROS buildfarm.

For this tutorial, we are going to create a simple hello world firmware.



```
{{#!wiki buildsystem catkin

<<TableOfContents(4)>>

<<Version(Indigo)>> You can now again build rosserial firmwares and other clients inside your regular ROS packages. This functionality is available from Indigo onward. The installation of the [[rosserial_arduino]] package now also installs [[http://packages.ubuntu.com/trusty/arduino-core|arduino-core]], so there's nothing additional needed to make this work.

== Making Your Project ==
Starting your rosserial_arduino project is just like creating any other package. In your catkin workspace's `src` folder:

{{{
catkin_create_pkg helloworld rosserial_arduino rosserial_client std_msgs
```



As usual, use `catkin_create_pkg` to create a package named
`helloworld`. You must depend on rosserial_arduino for the Arduino
toolchain, and on rosserial_client for client library generation macros.
And finally, since we are going to use
[std_msgs/msg/String](https://docs.ros.org/en/rolling/p/std_msgs/msg/String.html)
messages, you must depend on
[std_msgs](https://index.ros.org/p/std_msgs).

### Source Code

Copy the the source code below and make a file called
`firmware/chatter.cpp` in your helloworld package.



```cplusplus
#include <ros.h>
#include <std_msgs/String.h>

#include <Arduino.h>

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

void setup()
{
  nh.initNode();
  nh.advertise(chatter);
}

void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
}
```



This program is almost exactly the same as the hello world covered in
the
[Publisher Tutorial](#hello-world-example-publisher).

When you are compiling a cpp file outside of the Arduino IDE, you need
to explicitly include a header file which contains all of the Arduino
functions (digitalRead, analogRead, delay, etc.).

If you are unsure if you are going to be using a file with the Arduino
IDE versus CMake, just add this line at the top of your file. It never
hurts and it makes sure your file is always compatible with non-Arduino
IDE build systems.

### CMakeLists.txt

Open the CMakeLists.txt in your package directory and replace the
contents with the below:



```cplusplus
cmake_minimum_required(VERSION 2.8.3)
project(helloworld)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rosserial_arduino
  rosserial_client
)

catkin_package()

rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino
  SCRIPT make_libraries.py
)

rosserial_configure_client(
  DIRECTORY firmware
  TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

rosserial_add_client_target(firmware hello ALL)
rosserial_add_client_target(firmware hello-upload)
```



With [rosserial_client](https://index.ros.org/p/rosserial_client)'s
CMake scripts, we are not actually building the firmware directly, but
configuring a separate CMake project, and passing through targets from
the catkin package to the sub-project.

The `rosserial_generate_ros_lib` function creates a target called
helloworld_ros_lib, which will generate the rosserial client library,
including messages headers.

The `rosserial_configure_client` function creates a target which will
configure the CMake project in the specified subdirectory, optionally
using the supplied toolchain. In this case, we use the Arduino
toolchain, helpfully provided by
rosserial_arduino.

Finally, the `rosserial_add_client_target` calls each pass through
targets, so that when you run make the `helloworld_firmware_hello`
catkin target, it will configure the `firmware` directory and build
the `hello` target therein.

Now, we actually need a second `CMakeLists.txt`, and that's the one
for the firmware subproject. Create the file `firmware/CMakeLists.txt`
in your package, with the following contents:



```cplusplus
cmake_minimum_required(VERSION 2.8.3)

add_compile_options(-std=c++11)

include_directories(${ROS_LIB_DIR})

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
add_definitions(-DUSB_CON)

generate_arduino_firmware(hello
  SRCS chatter.cpp ${ROS_LIB_DIR}/time.cpp
  BOARD leonardo
  PORT /dev/ttyACM0
)
```



The `generate_arduino_firmware` function is provided by the
[arduino-cmake toolchain](https://github.com/queezythegreat/arduino-cmake),
which this uses. It handles the intricacies of locating Arduino, linking
any libraries you use, and so on.

You should be done!

### Build & Test

#### Using catkin_make

The firmware should build by default when you run `catkin_make`, but
you can also specify it explicitly:



```
catkin_make helloworld_firmware_hello
```



Now connect an Arduino Leonardo, confirm that it comes up as
`/dev/ttyACM0` (or change the firmware's `CMakeLists.txt`
accordingly), and program it:



```
catkin_make helloworld_firmware_hello-upload
```

 You might need to press the reset button to
reboot the Leonardo to allow code to be uploaded, and the port might
change after reset.

#### Using catkin tools

The firmware should build by default when you run `catkin build`, but
you can also specify it explicitly:



```
catkin build helloworld
```



Now connect an Arduino Leonardo, confirm that it comes up as
`/dev/ttyACM0` (or change the firmware's `CMakeLists.txt`
accordingly), and program it:



```
catkin build --no-deps  helloworld --make-args helloworld_firmware_hello-upload
```

 You might need to press the reset button to
reboot the Leonardo to allow code to be uploaded, and the port might
change after reset.

#### Testing

Now you can use the Arduino! Launch the following in separate terminals
to see it in action:



```
roscore
```





```
rosrun rosserial_python serial_node.py /dev/ttyACM0
```





```
rostopic echo chatter
```



### Additional Info

The CMake script which supplies these functions lives
[here](https://github.com/ros-drivers/rosserial/blob/indigo-devel/rosserial_client/cmake/rosserial_client-extras.cmake),
if you want to inspect it to understand better what is going on.

If there's a breakage you don't understand, a good first step is to
clean your workspace (delete build and devel trees), and then re-run in
verbose mode:



```
catkin_make VERBOSE=1
```



}}}}}



```
{{#!wiki buildsystem rosbuild

<<TableOfContents(4)>>

{{{#!wiki caution
'''Warning'''

Although we wont be using the Arduino IDE directly, you must have it installed.  This CMake build system uses the libraries and tools from the Arduino IDE installation.  Follow the [[rosserial_arduino/Tutorials/Arduino IDE Setup | Arduino IDE Setup Tutorial]] before continuing.
```



### Making Your Project

Making your rosserial_arduino project is just like making any other ROS
package. You create a package on your ROS_PACKAGE_PATH:



```
roscreate-pkg helloworld rosserial_arduino std_msgs
```

 As usual, use roscreate-pkg to create a
package named helloworld which depends on rosserial_arduino and
std_msgs. You must depend on rosserial_arduino and since we are going to
use std_msgs/String messages, the package depends on std_msgs.

### Source Code

Copy the the source code below and make a file called src/chatter.cpp in
your helloworld package.



```cplusplus
/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <WProgram.h>
#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[] = "hello world!";

void setup()
{
  nh.initNode();
  nh.advertise(chatter);
}

void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
}
```



This program is almost exactly the same as the hello world covered in
the
[Publisher Tutorial](#hello-world-example-publisher).
The only difference is

When you are compiling a cpp file outside of the Arduino IDE, you need
to explicitly include a header file which contains all of the Arduino
functions (digitalRead, analogRead, delay, etc.).

If you are unsure if you are going to be using a file with the Arduino
IDE versus CMake, just add this line at the top of your file. It never
hurts and it makes sure your file is always compatible with non-Arduino
IDE build systems.

### CMakeLists.txt

Open the CMakeLists.txt in your package directory and compare it to the
CMakeList below.



```cplusplus
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_find_ros_package(rosserial_arduino)
include(${rosserial_arduino_PACKAGE_PATH}/cmake_scripts/rosserial.cmake)
#====================================================================#
#  Chatter                                                           #
#====================================================================#


set(FIRMWARE_NAME chatter)

set(${FIRMWARE_NAME}_BOARD atmega328)         # Arduino Target board
set(${FIRMWARE_NAME}_SRCS src/chatter.cpp )
set(${FIRMWARE_NAME}_PORT /dev/ttyUSB0)            # Serial upload port
generate_ros_firmware(${FIRMWARE_NAME})
```



With rosserial_arduino, we are cross compiling our source code for the
AVR processor. As a result, we cannot use the standard ROS CMake file
directly. We take ROS's standard first two lines, delete the rest, and
then include the special rosserial_arduino cmake functions.

The included rosserial.cmake will setup the compiler, create the
generate_ros_firmware function, and attempt to find your Arduino
installation.

Next, the CMakeLists generates the ROS firmware. It does so using the
generate generate_ros_firmware function. This function uses a
declarative programming style. You describe your function in variables
named \${FIRMWARE}_\${SUFFIX}. The cpp files are stored in
chatter_SRCS. The Arduino's serial port is stored in chatter_PORT. The
arduino board type, atmega328, is stored chatter_BOARD. The board type
can be any type defined in hardware/arduino/boards.txt of your Arduino
installation. Typically this would be uno, mega2560, or atmega328.
generate_ros_firmware access these variables and creates your targets.
This setup is shown in

Once you set up the variables describing the firmware, it generates the
targets for compiling and programming that target. You can have as many
targets as you would like in a project. The compile target is always the
name of the firmware and the upload target is always the
TARGET_NAME-upload. We will be able to compile chatter by entering



```
make chatter
```

 and program an atmega328 type Arduino like the
Duemilanove using



```
make chatter-upload
```



Now, copy the above CMakeLists into your package CMakeLists.txt file and
then in the package directory run



```
cmake .
```



If you have you Arduino installation in a non-standard location, you
will receive the errors



```
-- The arduino sdk path is ARDUINO_SDK_PATH-NOTFOUND
-- REQUIRED_VARS  (missing:  ARDUINO_SDK_PATH ARDUINO_SDK_VERSION)
```

 when you go to generate your Makefile.
ARDUINO_SDK_PATH is the path to your Arduino IDE folder. To fix the
problem, you can either set the ARDUINO_SDK_PATH before including
rosserial.cmake



```
set(ARDUINO_SDK_PATH <path_to_arduino_installation>)
include(${rosserial_arduino_PACKAGE_PATH}/cmake_scripts/rosserial.cmake)
```



or move the installation to a folder on your application path.

### Testing

Connect your Arduino and make sure that the chatter_PORT corresponds to
the serial port of your Arduino. Program your Arduino using



```
make chatter-upload
```



Now you can use the Arduino! Start up the roscore ,the rosserial_python
serial_node.py , and rostopic echo to see it in action.



```
roscore
```





```
rosrun rosserial_python serial_node.py /dev/ttyUSB0
```





```
rostopic echo chatter
```



### Additional Info

#### generate_ros_firmware

The generate_ros_firmware macro takes care of all of the heavy lifting
necessary for your rosserial_arduino project. Make sure to define any
additional variables that you need.



```
# - Generate firmware for rosserial_arduino Devices
# generate_ros_firmware(TARGET_NAME)
#        TARGET_NAME - Name of target
# Creates a Arduino firmware target.
#
# The target options can be configured by setting options of
# the following format:
#      ${TARGET_NAME}${SUFFIX}
# The following suffixes are availabe:
#      _SRCS           # Sources
#      _HDRS           # Headers
#      _LIBS           # Libraries to linked in
#      _BOARD          # Board name (such as uno, mega2560, atmega328 ...)
#      _PORT           # Serial port, for upload and serial targets [OPTIONAL]
#      _AFLAGS         # Override global Avrdude flags for target
#      _SERIAL         # Serial command for serial target           [OPTIONAL]
#      _NO_AUTOLIBS    # Disables Arduino library detection
#      _NO_DEFAULT_COM # Disables a default communication implementation
```



#### Using Arduino Libraries

generate_ros_firmware will automatically link to any Arduino library
like Wire or Servo that is found in your Arduino IDE libraries folder.
It does not know about libraries in your sketchbook.

#### Finding rosserial_arduino cmake

rosserial_arduino CMake implementation is found under
rosserial_arduino/cmake_scripts. It is based on the
[arduino-cmake project](https://github.com/queezythegreat/arduino-cmake).
You should never need to edit these files. You just need to include them
into your CMakeLists.txt.

}}}}}

## Servo Controller Example

- **Description:** Tutorial for controlling an R/C servo with rosserial
  and an Arduino
- **Next Tutorial:**
  [IR Ranger](#ir-ranger-tutorial)

This tutorial explains how to control an R/C servo through ROS by using
an Arduino and rosserial. This can be used to control a release
mechanism, a cheap robot arm, ROS powered biped, or anything where you
need a cheap actuator. The code provided is a very basic example and
shows the control of a single hobby servo.

### Hardware

![attachment:arduino_servo.png](doc/arduino_servo.png)

This example assumes that you have an Arduino and a hobby r/c servo. The
r/c servo can be purchased from your local hobby shop,
[Towerhobbies](http://www3.towerhobbies.com/cgi-bin/wti0001p?&I=LXUK84&P=ML),
[Sparkfun](http://www.sparkfun.com/products/9064), etc.

The hobby servo r/c are great little actuators because they are
relatively cheap (as low as \$10) but contain a gear box and motor
control electronics. They are controlled by sending a squarewave pulse
of 1-2 milliseconds in width every 20 milliseconds. This typically moves
the servo arm from 0-180 degrees. Hobby servo's come in a huge variety
of sizes, torques, and angular precision.

### Code

The code for this tutorial is made extremely simple through the use of
the Arduino Servo library. The
[Servo Library](http://www.arduino.cc/en/Reference/Servo) handles all of
the low level control to generate and maintain the servo pulses. All
your code needs to do is specify the pin the servo is attached to and
then write the angle to the servo object. Underneath, the Servo library
uses the Arduino's built in timer interrupts to generate the correct
pulses. In this example, we only control one servo, but the same library
can be used to control up to 12 servos on most Arduino boards and 48 on
the Arduino Mega.



```cplusplus
/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh;

Servo servo;

void servo_cb( const std_msgs::UInt16& cmd_msg){
  servo.write(cmd_msg.data); //set servo angle, should be from 0-180  
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}


ros::Subscriber<std_msgs::UInt16> sub("servo", servo_cb);

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  
  servo.attach(9); //attach it to pin 9
}

void loop(){
  nh.spinOnce();
  delay(1);
}
```



The key servo specific areas here are the fact that we made a global
Servo object, attached to the correct arduino pin, and then on every
servo topic call back, we write the servos new angle to the servo
object.

### Testing

First, startup your roscore and the rosserial python node in their own
terminal windows. 

```
roscore
```





```

```

}}} 

```

```

}}}

Now, in a new terminal window, use rostopic pub to control your servo!
Simply specify the angle from 0-180 and watch it move.



```
rostopic pub servo std_msgs/UInt16  <angle>
```

## IR Ranger Tutorial

- **Note:** This tutorial assumes that you have completed the previous
  tutorials:
  [Arduino IDE Setup](#arduino-ide-setup).
- **Description:** Using an IR Ranger with rosserial and an Arduino
- **Tutorial Level:** BEGINNER
- **Next Tutorial:**
  [SRF08 Ultrasonic Range Finder](#srf08-ultrasonic-ranger)

[rosserial](https://index.ros.org/p/rosserial) allows you to easily
integrate Arduino-based hardware with ROS. This tutorial will explain
how to use a sharp IR ranger with an Arduino.

You will need an [Arduino](http://www.arduino.cc) which you can purchase
[here](http://www.sparkfun.com/products/9950) for example. Additionally,
you will need a
[Sharp IR Ranger, Model# GP2D120XJ00F](http://www.sparkfun.com/products/8959),
and a way to connect your IR Ranger to your Arduino such as a breadboard
or protoboard. If you have a different model IR Ranger, you'll need to
modify the code based on your model. This is a helpful
[Arudino library](https://github.com/guillaume-rico/SharpIR) for some
other popular models.

### Hardware Setup

To get started, connect your ranger to your Arduino as shown below. Make
sure to connect the signal pin of the ranger to *analog input 0*.

![width=600 height=](doc/arduino_ir_ranger.jpg)

### Software Setup

#### The Code

Next, open up your Arduino IDE and copy in the code below. The code can
also be found in the `rosserial_arduino_demos` package under
'sketches\\IrRanger'.



```cplusplus
/* 
 * rosserial IR Ranger Example  
 * 
 * This example is calibrated for the Sharp GP2D120XJ00F.
 */

#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>

ros::NodeHandle  nh;
sensor_msgs::Range range_msg;
ros::Publisher pub_range( "range_data", &range_msg);

const int analog_pin = 0;
unsigned long range_timer;

/*
 * getRange() - samples the analog input from the ranger
 * and converts it into meters.  
 * 
 * NOTE: This function is only applicable to the GP2D120XJ00F !!
 * Using this function with other Rangers will provide incorrect readings.
 */
float getRange(int pin_num){
    int sample;
    // Get data
    sample = analogRead(pin_num)/4;
    // if the ADC reading is too low, 
    //   then we are really far away from anything
    if(sample < 10)
        return 254;     // max range
    // Magic numbers to get cm
    sample= 1309/(sample-3);
    return (sample - 1)/100; //convert to meters
}

char frameid[] = "/ir_ranger";

void setup()
{
  nh.initNode();
  nh.advertise(pub_range);
  
  range_msg.radiation_type = sensor_msgs::Range::INFRARED;
  range_msg.header.frame_id =  frameid;
  range_msg.field_of_view = 0.01;
  range_msg.min_range = 0.03;  // For GP2D120XJ00F only. Adjust for other IR rangers
  range_msg.max_range = 0.4;   // For GP2D120XJ00F only. Adjust for other IR rangers
}

void loop()
{
  // publish the range value every 50 milliseconds
  //   since it takes that long for the sensor to stabilize
  if ( (millis()-range_timer) > 50){
    range_msg.range = getRange(analog_pin);
    range_msg.header.stamp = nh.now();
    pub_range.publish(&range_msg);
    range_timer =  millis();
  }
  nh.spinOnce();
}
```



#### The Code Explained

Now let's break down the code. As always, the code begins by including
the appropriate message headers and `ros.h` from the rosserial library
and then instantiating the publisher. We then define the `analog_pin`
that the ranger is attached to, creates a timer variable, and defines a
function that converts the analog signal to a corresponding distance
reading in meters. Here, the code creates a global variable for the
sensors frame id string. It is important to make this string global so
it will be alive for as long as the message will be in use. In the
Arduino's setup function, the code initializes the node handle and then
fills in the descriptor fields for `range_msg`. This particular IR
Ranger reads between 3cm and 40cm.

Note: Many IR libraries report in centimeters by default. You may need
to convert their readings to meters if you're using a different Ranger
or IR helper library. Finally, in the publish loop, the Arduino samples
the ranger once every 50 milliseconds and publishes the range data.

### Launching the App

After you program your Arduino, its time to visualize the sensors
measurements using rxplot.



```
roscore
```





```

```

}}} 

```

```

}}}



```
If this is the first time you've run rqt_plot, you will need to [[rqt_plot#First_time_running| install the package first]].
```

}}}

## SRF08 Ultrasonic Ranger

- **Description:** In this tutorial, we will use an Arduino and a
  [SRF08 Ultrasonic Ranger](http://www.acroname.com/robotics/parts/R145-SRF08.html)
  as a Range Finder.The SRF08 communicates with an Arduino over SPI/I2C.
- **Tutorial Level:** BEGINNER
- **Next Tutorial:**
  [BlinkM](#blinkm-tutorial)

[rosserial](https://index.ros.org/p/rosserial) allows you to easily
integrate Arduino-based hardware with ROS. This tutorial will explain
how to use a SRF08 Ultrasonic ranger with an Arduino.

You will need an [Arduino](http://www.arduino.cc) ,a
[SRF08 Ultrasonic Ranger](http://www.acroname.com/robotics/parts/R145-SRF08.html),
and a way to connect your Ranger to your Arduino such as a breadboard or
protoboard.The SRF08 Arduino library can be downloaded
from[Sonar_srf08.zip](doc/Sonar_srf08.zip)

### Hardware Setup

Below is the diagram for setting up the SRF08 sensor for a typical
Arduino. The only additional circuitry necessary are two 1.8kOhm pullup
resistors as seen below.

. ![width="600 height="](doc/SRF08_CIRCUIT.png)

### The Code

Next, open up your Arduino IDE and copy in the code below.



```cplusplus
/*
 * rosserial SRF08 Ultrasonic Ranger Example
 *
 * This example is calibrated for the SRF08 Ultrasonic Ranger.
 */
#include <Sonar_srf08.h> //SRF08 specific library
#include <WProgram.h>
#include <Wire.h>
#include <ros.h>
#include <std_msgs/Float32.h>


//Set up the ros node and publisher
std_msgs::Float32 sonar_msg;
ros::Publisher pub_sonar("sonar", &sonar_msg);
ros::NodeHandle nh;


Sonar_srf08 MySonar; //create MySonar object

#define CommandRegister 0x00
int New_Address = 248; //0xF8
#define ResultRegister  0x02

float sensorReading =0;

char unit = 'i'; // 'i' for inches , 'c' for centimeters


void setup()
{
  MySonar.connect();
  MySonar.changeAddress(CommandRegister, New_Address);
  New_Address += 4;
  nh.initNode();
  nh.advertise(pub_sonar);

}


long publisher_timer;

void loop()
{

  if (millis() > publisher_timer) {

   // step 1: request reading from sensor
   MySonar.setUnit(CommandRegister, New_Address, unit);

   //pause
  delay(70);

  // set register for reading
  MySonar.setRegister(New_Address, ResultRegister);

  // read data from result register
  sensorReading = MySonar.readData(New_Address, 2);

  sonar_msg.data = sensorReading;
  pub_sonar.publish(&sonar_msg);

  publisher_timer = millis() + 4000; //publish once a second

  }

  nh.spinOnce();
}
```

 The special bit of code in this example is the
use of Arduino's
[Wire library](http://www.arduino.cc/en/Reference/Wire). Wire is a I2C
library that simplifies reading and writing to the I2C bus.

### Launching the App



```
roscore
```





```

```

}}} 

```

```

}}}



```
rostopic echo sonar
```

## BlinkM Tutorial

- **Description:** Control a BlinkM through ROS using an Arduino and
  rosserial
- **Tutorial Level:** ADVANCED
- **Next Tutorial:**
  [Arduino Oscilloscope](#arduino-oscilloscope)

One thing every robot needs is a good indicator light. If you are
looking for something more fancy or just plane brighter than just a
single tiny LED, the BlinkM is a good choice for your robotics project.
The [BlinkM](http://thingm.com/products/blinkm) is a I2C controlled
multi-colored LED which can change colors and run lighting scripts. In
this tutorial, we are going to use the default scripts of a BlinkM to
create a multicolored blinking or solid indicator light.

![attachment:blinkm.jpg](doc/blinkm.jpg)

For this tutorial, you will be using the example provided with ros_lib.
In your Arduino IDE, go to File>Examples>ros_lib>BlinkM. The code can
also be found in rosserial_arduino/src/ros_lib/examples.

For a full reference and getting started guide for the BlinkM, see the
[BlinkM datasheet](http://docs.google.com/viewer?url=http%3A%2F%2Fthingm.com%2Ffileadmin%2Fthingm%2Fdownloads%2FBlinkM_datasheet.pdf).

### Hardware

![attachment:blinkm_connections.jpg](doc/blinkm_connections.jpg)

[Diagram from the BlinkM datasheet](http://docs.google.com/viewer?url=http%3A%2F%2Fthingm.com%2Ffileadmin%2Fthingm%2Fdownloads%2FBlinkM_datasheet.pdf)

The hardware for this tutorial is relatively simple. All that is
neccesary is a BlinkM and an Arduino. The BlinkM needs to be connected
to 5V, GND, and the Arduino's I2C connections (SDA and SCL). You can
purchase both a [BlinkM](http://www.sparkfun.com/products/8579) and an
[Arduino](http://www.sparkfun.com/products/9950) from Sparkfun.

### Code

The ROS serial integration code for the blinkm is below. In this arduino
sketch, the node subscribes to a
[std_msgs/msg/String](https://docs.ros.org/en/rolling/p/std_msgs/msg/String.html)
on the blinkm topic. In the callback, the the node parses the command to
determine the LED's color and if it should be blinking. The LED can be
red (r), blue (b), magenta(m), green(g), white(w), cyan(c), and
yellow(y). If the command begins with an 'S' or 's' the LED is a
solid color. Otherwise, the LED blinks that color.



```cplusplus
/*
*  RosSerial BlinkM Example
*  This program shows how to control a blinkm
*  from an arduino using RosSerial
*/

#include "WProgram.h" //include the Arduino library
#include <stdlib.h>


#include <ros.h>
#include <std_msgs/String.h>


//include Wire/ twi for the BlinkM
#include <Wire.h>
extern "C" { 
#include "utility/twi.h" 
}

#include "BlinkM_funcs.h"
const byte blinkm_addr = 0x09; //default blinkm address


void setLED( bool solid,  char color)
{

       	if (solid)
	{
           switch (color)
		{

		case 'w':  // white
			BlinkM_stopScript( blinkm_addr );
			BlinkM_fadeToRGB( blinkm_addr, 0xff,0xff,0xff);  
			break;
			
		case 'r': //RED
			BlinkM_stopScript( blinkm_addr );
			BlinkM_fadeToRGB( blinkm_addr, 0xff,0,0);  
			break;

		case 'g':// Green
			BlinkM_stopScript( blinkm_addr );
			BlinkM_fadeToRGB( blinkm_addr, 0,0xff,0);
			break;

		case 'b':// Blue
			BlinkM_stopScript( blinkm_addr );
			BlinkM_fadeToRGB( blinkm_addr, 0,0,0xff);
			break;

		case 'c':// Cyan
			BlinkM_stopScript( blinkm_addr );
			BlinkM_fadeToRGB( blinkm_addr, 0,0xff,0xff);
			break;

		case 'm': // Magenta
			BlinkM_stopScript( blinkm_addr );
			BlinkM_fadeToRGB( blinkm_addr, 0xff,0,0xff);
			break;

		case 'y': // yellow
                        BlinkM_stopScript( blinkm_addr );
			BlinkM_fadeToRGB( blinkm_addr, 0xff,0xff,0);
			break;

		default: // Black
			BlinkM_stopScript( blinkm_addr );
			BlinkM_fadeToRGB( blinkm_addr, 0,0,0);
			break;
		}
	}


	else
	{
               	switch (color)
		{
		case 'r':  // Blink Red
                        BlinkM_stopScript( blinkm_addr );
			BlinkM_playScript( blinkm_addr, 3,0,0 );
			break;
		case 'w':  // Blink white
                        BlinkM_stopScript( blinkm_addr );
			BlinkM_playScript( blinkm_addr, 2,0,0 );
			break;
		case 'g':  // Blink Green
			BlinkM_stopScript( blinkm_addr );
			BlinkM_playScript( blinkm_addr, 4,0,0 );
			break;

		case 'b': // Blink Blue
			BlinkM_stopScript( blinkm_addr );
			BlinkM_playScript( blinkm_addr, 5,0,0 );
			break;

		case 'c': //Blink Cyan
			BlinkM_stopScript( blinkm_addr );
			BlinkM_playScript( blinkm_addr, 6,0,0 );
			break;

		case 'm': //Blink Magenta
			BlinkM_stopScript( blinkm_addr );
			BlinkM_playScript( blinkm_addr, 7,0,0 );
			break;

		case 'y': //Blink Yellow
                        BlinkM_stopScript( blinkm_addr );
			BlinkM_playScript( blinkm_addr, 8,0,0 );
			break;

		default: //OFF
			BlinkM_stopScript( blinkm_addr );
			BlinkM_playScript( blinkm_addr, 9,0,0 );
			break;
		}

	}
}

void light_cb( const std_msgs::String& light_cmd){
        bool solid =false;
        char color; 
        if (strlen( (const char* ) light_cmd.data) ==2 ){
          solid  = (light_cmd.data[0] == 'S') || (light_cmd.data[0] == 's');
          color = light_cmd.data[1];
        }
        else{
          solid=  false;
          color = light_cmd.data[0];
        } 
        
	setLED(solid, color);
}



ros::NodeHandle  nh;
ros::Subscriber<std_msgs::String> sub("blinkm" , light_cb);


void setup()
{
   
    pinMode(13, OUTPUT); //set up the LED

	BlinkM_beginWithPower();
	delay(100);
	BlinkM_stopScript(blinkm_addr);  // turn off startup script
	setLED(false, 0); //turn off the led
        
        nh.initNode();
        nh.subscribe(sub);

}

void loop()
{
 nh.spinOnce();
 delay(1);
}
```



Another key feature of this script that should be noted is that the I2C
address of the BlinkM is set to the default BlinkM I2C. If yours has
been reprogrammed, or if you want to control multiple BlinkMs, you will
need to change this address.

### Testing

Program you Arduino with the BlinkM sketch. Open the blinkm sketch from
the package:*roserial_arduino_tutorials/sketches/BlinkM folder and
program your Arduino.*

Start up the roscore in a new termianl 

```
roscore
```



Launch the rosserial_python serial_node. Make sure to choose the right
serial port. 

```

```

}}} 

```

```

}}}

Now, look at the red blinking light! 

```
rostopic pub blinkm std_msgs/String "br"
```



Blue light! 

```
rostopic pub blinkm std_msgs/String "sb"
```

## Arduino Oscilloscope

- **Description:** In this tutorial, we will be making a poor man's
  oscilloscope using the Arduino's ADC (analog to digital converter)
  and rqt_plot.
- **Tutorial Level:** ADVANCED
- **Next Tutorial:**
  [NodeHandle and ArduinoHardware](#advanced-configuration-for-nodehandle-and-arduinohardware)

In this tutorial, we will be making a poor man's oscilloscope using the
Arduino's ADC (analog to digital converter) and rqt_plot. We will set
up a publisher which will forward the analog values from each of the
Arduino's 6 ADC pins to ROS using rosserial. This will be a relatively
bad oscilloscope (very low sampling frequency and resolution), but it
will be perfect for many quick and dirty setups.

The code for this tutorial can be found in your Arduino IDE at
File>Examples>ros_lib>ADC

If you have not done so already, make sure to clone the rosserial stack
and add it to your ROS_PACKAGE_PATH.

### Custom Message

This code uses a custom message type, the `ADC.msg`. This message has
`uint16` field for each of the analog input pins on the standard
Arduino.



```
uint16 adc0
uint16 adc1
uint16 adc2
uint16 adc3
uint16 adc4
uint16 adc5
```



This message is a part of the rosserial_arduino package and has been
added to your ros_lib library.

### The Code

The code for this tutorial is fairly straight forward. The code
registers a `Adc` msg publisher, initializes the node handle, and then
tries to publish as many `Adc` messages as possible.



```cplusplus
/*
 * rosserial ADC Example
 *
 * This is a poor man's Oscilloscope.  It does not have the sampling
 * rate or accuracy of a commerical scope, but it is great to get
 * an analog value into ROS in a pinch.
 */

#include <WProgram.h>
#include <ros.h>
#include <rosserial_arduino/Adc.h>

ros::NodeHandle nh;

rosserial_arduino::Adc adc_msg;
ros::Publisher p("adc", &adc_msg);


void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();

  nh.advertise(p);
}

//We average the analog reading to eliminate some of the noise
int averageAnalog(int pin){
 int v=0;
      for(int i=0; i<4; i++) v+= analogRead(pin);
      return v/4;
}

long adc_timer;

void loop()
{

    adc_msg.adc0 = averageAnalog(0);
    adc_msg.adc1 = averageAnalog(1);
    adc_msg.adc2 = averageAnalog(2);
    adc_msg.adc3 = averageAnalog(3);
    adc_msg.adc4 = averageAnalog(4);
    adc_msg.adc5 = averageAnalog(5);

    p.publish(&adc_msg);

  nh.spinOnce();
}
```



### Running the Oscilloscope



```
roscore
```





```

```

}}} 

```

```

}}}



```
rqt_plot adc/adc0
```

 Now, you should see the raw analog value on
adc0 displayed in [rqt_plot](https://index.ros.org/p/rqt_plot). The
reported readings are going to be from 0-1024 because the Arduino has a
10-bit ADC. In order to get the actual voltage, adc_val / 1024 \*
Arduino supply voltage. Try connecting ADC0 to ground, 5V, and 3.3V on
your arduino and observing the value. Also notice that when you don't
have anything connected to the ADC, it is a floating analog input and
its value will oscillate randomly.

For example, look at the below output where the ADC0 input is switched
between 5V and gnd. There are wild oscillations during the transition.

![attachment:RXPlot.jpg](doc/RXPlot.jpg)

## Advanced Configuration for NodeHandle and ArduinoHardware.

- **Note:** This tutorial assumes that you have completed the previous
  tutorials:
  [Hello World (example Publisher)](#hello-world-example-publisher).
- **Description:** This tutorial shows step by step how to configure
  NodeHandle and ArduinoHareware to better suit user needs.
- **Tutorial Level:** ADVANCED

The `rosserial_arduino` package is designed to work out-of-box for
many users, but users might need to change the `ros.h` and the
`ArduinoHardware.h` to better suit their need.

In this tutorial, we will be changing both the file to demonstrate some
of the advanced settings.

### Prerequisite

#### Hardware

- [Arduino Mega](https://store.arduino.cc/usa/arduino-mega-2560-rev3)
- [FTDI Breakout board](https://www.sparkfun.com/products/9716) or
  similar UART to USB bridge
- USB cables for both devices
- Some jumper wire to connect both devices

#### Objectives

##### Publisher/Subscriber Numbers and Input/Output Buffers

Arduino Mega has 8 kB of SRAM, much more than most of the other Arduino
boards (Uno has 2 kB, Leonardo has 2.5 kB). This enable us to have more
Publishers/Subscribers and use bigger messages.

While we might never need change those numbers for Arduino Mega as the
default value is plenty enough, for smaller board that has limited SRAM,
we would need to reduce the memory usage.

For the sake for demonstrating how to do it, we will decrease the number
of publisher/subscriber from 25/25 to 10/15, and the input/output buffer
size for 512/512 bytes to 128/256 bytes.

##### Change the Serial Port

Arduino Mega has 4 serial port in total:

  -- ----------------- -- ------------ -- ------------ --
     **Object Name**      **RX Pin**      **TX Pin**   
     Serial               0               1            
     Serial1              19              18           
     Serial2              17              16           
     Serial3              15              14           
  -- ----------------- -- ------------ -- ------------ --

Normally, we use `Serial` to communicate with Arduino Mega, as
`Serial` is connected to the UART to USB bridge, and shows as the port
we see on the computer. But for this tutorial, we will be using the
`Serial1`. Because the `Serial1` can not communicate directly with
the computer, we use a UART to USB bridge
([FTDI board](https://www.sparkfun.com/products/9716)).

##### Change the Baudate

By default the rosserial uses `57600` as the
[baudrate](https://www.mathworks.com/help/matlab/matlab_external/baudrate.html)
for communication. This rate is not the maximum that Arduino can
achieve, and when there is a lot of information that need to sent back
to ROS, the baudrate can become the bottleneck.

The Arduino Mega that operate at 16 Mhz can communicate wit baudrate up
to 2M bits per seconds (bps) and other boards such as
[Teensy 3.2](https://www.pjrc.com/teensy/td_uart.html) can go up to 4.6
Mbps. The inefficiency of Arduino Serial library quickly become the
bottleneck at high baudrate and we can't really benefit from the
increased baudrate. (more information on this
[here](https://arduino.stackexchange.com/questions/296/how-high-of-a-baud-rate-can-i-go-without-errors))

For this tutorial, we will be changing the baudrate from 57.6 Kbps to
500 Kbps.

### The code

We will have 3 source files for this example. The main sketch, the
`ros.h` and the `ArduinoHardware.h`. Because many of our changes are
under the hood and do not affect how we use rosserial, the hello world
sketch is used with minimal modification. The original `ros.h` and
`ArduinoHardware.h` can be found either in the `src/ros_lib/` of
`rosserial_arduino` package, or in the generated `ros_lib` in your
Arduino library. You can modify them directly to achieve the same effect
but it is recommended to add them to your sketch folder, to make the
change per sketch and persistent over regeneration of `ros_lib`.

#### Main Sketch

We will first create a new sketch and copy the following code to it:


```cplusplus
/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include "ros.h" 
#include <std_msgs/String.h>

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

void setup()
{
  nh.initNode();
  nh.advertise(chatter);
}

void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
}
```



#### ros.h

`ros.h` is the `NodeHandle` definition, and we will create a new tab
in Arduino by click on the down arrow icon below Serial Monitor, name it
`ros.h`, and copy the following content to it:


```cplusplus
#ifndef _ROS_H_
#define _ROS_H_

#include "ros/node_handle.h"
#include "ArduinoHardware.h"

namespace ros
{
  // default is 25, 25, 512, 512  
  typedef NodeHandle_<ArduinoHardware, 10, 15, 128, 256> NodeHandle;

  // This is legal too and will use the default 25, 25, 512, 512
  //typedef NodeHandle_<ArduinoHardware> NodeHandle;
}

#endif
```



#### ArduinoHardware.h

`ArduinoHardware.h`, hardware definition for Arduino platform,
similarly to `ros.h`, we will create a new tab named
`ArduinoHardware.h` and copy the following code to it:


```cplusplus
#ifndef ROS_ARDUINO_HARDWARE_H_
#define ROS_ARDUINO_HARDWARE_H_

#if ARDUINO>=100
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

#include <HardwareSerial.h>
#define SERIAL_CLASS HardwareSerial

class ArduinoHardware
{
  public:
    ArduinoHardware()
    {
      iostream = &Serial1;
      baud_ = 500000;
    }
  
    void setBaud(long baud)
    {
      this->baud_= baud;
    }
  
    int getBaud()
    {
      return baud_;
    }

    void init()
    {
      iostream->begin(baud_);
    }

    int read()
    {
      return iostream->read();
    };

    void write(uint8_t* data, int length)
    {
      for(int i=0; i<length; i++)
      {
        iostream->write(data[i]);
      }
    }

    unsigned long time()
    {
      return millis();
    }

  protected:
    SERIAL_CLASS* iostream;
    long baud_;
};

#endif
```



### The Code Explained

#### The Main Sketch

The only difference that we found it the main sketch is how `ros.h` is
included, the brackets `<ros.h>` in the original `hello world` are
replaced by quotation marks `"ros.h"`. This tells the compiler to
look for the file in local path before search it in other libraries.
Everything else are the same as the hello world example.

#### ros.h

This set our `NodeHandle` to have 10 Publishers, 15 Subscriber, 128
bytes for input buffer and 256 bytes for output buffer.

#### ArduinoHardware.h

Here we include the header for the serial object we communicate with.
The serial object class must have `read()`, `write()` and
`begin(baudrate)` member function implemented. `Serial`, and
`Serial1` are both `HardwareSerial` objects. Here we create the
default constructor for `ArduinoHardware` and set the serial port we
use to `Serial1` and baudrate to 500 kbps.

### Compile and Upload the Code

The code is compile and upload as usual. After compilation, you will be
able to see a message of the code like this 

```
Sketch uses 9,552 bytes (3%) of program storage space. Maximum is 253,952 bytes.
Global variables use 1,130 bytes (13%) of dynamic memory, leaving 7,062 bytes for local variables. Maximum is 8,192 bytes.
```

 By
varying the buffer sizes and Publishers/Subscribers numbers, the dynamic
memory usage will change as well.

### Running the Code

We will be using `Serial1` to communicate with ROS now. Disconnect the
USB cable from Arduino, Connect the FTDI breakout board to the Arduino
Mega as following: 

```
FTDI     Arduino Mega
VCC  ->  5V
TXO  ->  Pin 19
RXI  ->  Pin 18
GND  ->  GND
```



Connect the FTDI breakout board to the computer using USB cable.

Now, launch the [roscore](roscore) in a new terminal window:


```
roscore
```



Next, run the rosserial client application that forwards your Arduino
messages to the rest of ROS. Make sure to use the correct serial port:


```
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=500000
```



Finally, watch the greetings come in from your Arduino by launching a
new terminal window and entering : 

```
rostopic echo chatter
```



### Extras

`ros.h` and `ArduinoHardware.h` license. 

```cplusplus
/* 
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote prducts derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
```
