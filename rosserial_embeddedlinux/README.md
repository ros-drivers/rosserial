## Introduction

There are a variety of great embedded linux systems on the market today
which enable quickly and easily programming hardware. Some, like the
[VEXPro Arm9 Microcontroller](http://www.vexrobotics.com/products/vexpro/217-2180.html)
support an entire electro-mechanical robotics platform. Others, like the
Chumby alarm clock or WRT54-class routers provide just an inexpensive
linux controller. This class of device typically supports USB and wifi,
has USB drivers for webcam and USB-serial dongles, is physically small,
and consumes under 10 watts of electrical power. This makes them
expandable and interesting for use on smaller robots, especially when
vision is desired.

Using the
rosserial_embeddedlinux package,
you can use ROS directly with the these systems.
[rosserial](https://index.ros.org/p/rosserial) provides a ROS
communication protocol that works over your embedded linux system's
serial UART, or its wifi or network connection. It allows your embedded
linux system to run linux processes that are full fledged ROS nodes that
can directly publish and subscribe to ROS topics, advertise services and
request services, publish TF transforms, and get the ROS system time
over any of the supported connection types.

There are three environments you'll have to deal with when using an
embedded linux system: ROS, the embedded system development environment,
and the embedded system itself. These are depicted below.

![attachment:rosserial_embeddedlinux_schematic.png](doc/rosserial_embeddedlinux_schematic.png)

Rosserial_python presents the embedded linux application(s) as a ROS
node(s), and proxies messages between embedded linux system and the rest
of ROS.

The rosserial_embeddedlinux
package supports the following major connection types and capabilities:

- TCP/IP or serial connection between ROS system and embedded linux
  system supports wifi, wired ethernet, USB-serial dongle, wired serial,
  etc.
- TCP/IP connection supports reconnect after disconnect (e.g. in case
  robot wanders out of range of wifi, or program on robot is restarted).
- TCP/IP connection supports multiple connections from processes on
  embedded linux systems. This supports both multiple processes on one
  robot (e.g. a vision process and a robot motion process), and multiple
  robots (or both).

## Purpose of Package

This package contains embedded-linux-specific extensions required to run
[rosserial](https://index.ros.org/p/rosserial) on a small embedded linux
system such as the VEXPro controller, Chumby alarm clock, WRT54GL
router, Raspberry Pi, or many similar devices. It is meant to
demonstrate how easy it is to integrate custom hardware and cheap
sensors including USB cameras into your ROS project using an embedded
linux system. The Tutorials of this package will walk you through
setting up your run environment and creating a few example programs.

Using this package may not be appropriate if your embedded linux system
can run Ubuntu and the full ROS stacks; it is really aimed at smaller or
non-x86/Ubuntu linux systems.

## Prerequisites

You should have a development toolchain available to you that is capable
of compiling C++ and downloading it to your embedded linux system, and
know how to use it. Instructions are given for installing and using
toolchains for specific platforms, as well as generic procedures that
can be adapted to suit all platforms.

## Tutorials

Go to the [tutorials](#tutorials-1)
to learn how to install and use the package to connect your embedded
linux system to ROS.

## Bugs

Please file new bugs on the project's
[bug tracker](https://github.com/ros-drivers/rosserial/issues). The bugs
listed below are the most serious, which you would want to be aware of
when considering use of this technology. You are encouraged to
contribute fixes. 1. Service client on the embedded system doesn't work
reliably. (Publisher, Subscriber, and Service server are good.) This is
true on both arduino and embeddedlinux packages. 1. Parameters are not
supported on the embedded system. 1. APIs are not thread-safe

## Tutorials

The tutorials below show programs you can run on your embedded linux
system that interact over the network (or serial port, if that's what
you're using) with ROS running on your ROS workstation. You will need
to modify each program to change the hard-coded IP address and replace
it with the particular network name or address (or serial port name) of
the system running the rosserial_python proxy on your network.

- [rosserial_embeddedlinux Advanced Techniques](#rosserial_embeddedlinux-advanced-techniques):
  This tutorial describes several advanced techniques for debugging
  problems or running or building the package different ways
- [Hello Ros (example publisher)](#hello-ros-example-publisher):
  This tutorial shows step by step how to create an example publisher
- [Hello Embedded Linux (example subscriber)](#hello-embedded-linux-example-subscriber):
  This tutorial shows how to make an embedded linux application
  subscribe to a topic published in ROS.
- [Example service](#example-service):
  This tutorial shows how to make an embedded linux application offer a
  service to ROS service clients
- [Installing rosserial_embeddedlinux](#installing-rosserial_embeddedlinux):
  This tutorial shows how to set up the ROS environment, and gives some
  hints to consider when setting up your embedded linux development
  environment.
- [VEXPro range motor control loop](#vexpro-range-motor-control-loop):
  This tutorial describes how to use interrupts to accurately measure
  range using an ultrasonic rangefinder, then goes on to apply that in a
  motor control system with the loop controller running in a ROS node.
- [VEXPro servo position control example](#vexpro-servo-position-control-example):
  This tutorial shows how to use the VEXPro embedded linux system with
  the libqwerk library that exposes device I/O to applications, in order
  to control the position of a servo.

## rosserial_embeddedlinux Advanced Techniques

- **Note:** This tutorial assumes that you have completed the previous
  tutorials:
  [rosserial design notes](https://index.ros.org/p/rosserial).
- **Description:** This tutorial describes several advanced techniques
  for debugging problems or running or building the package different
  ways
- **Tutorial Level:** ADVANCED

### rosserial_embeddedlinux Design Notes

rosserial_python is designed to be a TCP/IP server, when invoked with
the tcp argument. It uses the unix idiom of waiting for a socket
connection, then fork/exec-ing the server process to do the requested
work while the process parent goes back to waiting for more connection
requests. Python multiprocessing is used as the fork technique, but it
may not have all the typical unix semantics such as the controlling tty.
The program does not exec an external binary - the child handles
completing the connection to the ephemeral port and handling messages
to/from the client.

### Debugging rosserial_python with print statements

Sometimes you need to find out what the python code is doing. A
breakpoint can be inserted and the program put into single-step mode by
inserting this code at the desired breakpoint location:


```
import pdb; pdb.set_trace()
```



It's often difficult to inject breakpoints and have the system stay
running in a sane manner - there are timers that will cause the system
to retrigger topic exchanges, time, etc. Injecting print statements can
be a better approach to understanding what the code is doing.

A consequence of the fork-exec idiom described in the design notes is
that the output of print statements is not visible. rosserial_python
addresses this by providing a mode of running where it doesn't
fork-exec the process. This is controlled by the boolean parameter
fork_server, which defaults to True.

To get print statement output to print to your terminal, start
rosserial_python with fork_server equal to False by setting the
/rosserial_embeddedlinux/fork_server parameter as follows:


```
rosparam set /rosserial_embeddedlinux/fork_server False
rosrun rosserial_python serial_node.py tcp
```

 A consequence of starting this way is that only
one embedded linux client at a time will be able to connect to the
rosserial_python proxy.

When you're done debugging, remember to delete the parameter:


```
rosparam delete /rosserial_embeddedlinux/fork_server
```



### Building rosserial_embeddedlinux as a library

You may want to build the rosserial_embeddedlinux module as a library
you can link applications against (statically or dynamically). The
Makefile below builds it as a static library using the VEXPro toolset.
It may be out of date - be forewarned. It is presented as a guide to
creating a library. Adjust for your toolchain.



```
# source files.
CPPSRC =  	time.cpp duration.cpp
SRC =  	embedded_linux_comms.c

CPPOBJ = $(CPPSRC:.cpp=.o)
OBJ = $(SRC:.c=.o)

OUT = librosembeddedlinux.a

# include directories
INCLUDES = -I/usr/local/terkos/arm/arm-oe-linux-uclibcgnueabi/include \
	-I/usr/local/terkos/arm/arm-oe-linux-uclibcgnueabi/usr/include \
	-I/usr/local/terkos/arm/arm-oe-linux-uclibcgnueabi/include/terk \
	-I. -I..

# C++ compiler flags (-g -O2 -Wall)
CCFLAGS = -O0 -g3 -Wall -c -fmessage-length=0 -DBUILD_LIBROSEMBEDDEDLINUX

# compiler
CCC = /usr/local/terkos/arm/arm-oe-linux-uclibcgnueabi/bin/g++
CC = /usr/local/terkos/arm/arm-oe-linux-uclibcgnueabi/bin/gcc

# library paths
LIBS = -L../ -L/usr/local/lib -lm

# compile flags
LDFLAGS = -g

.SUFFIXES: .cpp .c

default: $(OUT)

.c.o:
	$(CC) $(INCLUDES) $(CCFLAGS) -c $< -o $@

.cpp.o:
	$(CCC) $(INCLUDES) $(CCFLAGS) -c $< -o $@

$(OUT): $(CPPOBJ) $(OBJ)
	ar rcs $(OUT) $(OBJ) $(CPPOBJ)

clean:
	rm -f $(OBJ) $(CPPOBJ) $(OUT) Makefile.bak
```

## Hello Ros (example publisher)

- **Note:** This tutorial assumes that you have completed the previous
  tutorials:
  [Install rosserial_embeddedlinux](#installing-rosserial_embeddedlinux).
- **Description:** This tutorial shows step by step how to create an
  example publisher
- **Tutorial Level:** BEGINNER
- **Next Tutorial:**
  [Hello Embedded Linux (example subscriber)](#hello-embedded-linux-example-subscriber)
  We'll start our exploration into rosserial for embedded linux by
  creating a "Hello Ros" program for our embedded linux box.

### The Code

Use your favorite editor or IDE to open the !HelloRos tutorial code
which is found in examples/HelloRos/HelloROS.cpp. You should see the
following:



```cplusplus
/*
 * rosserial Publisher Example
 * Prints "Hello ROS!"
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <stdio.h>

ros::NodeHandle  nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char *rosSrvrIp = "192.168.15.121";
char hello[13] = "Hello ROS!";

int main()
{
	//nh.initNode();
	nh.initNode(rosSrvrIp);
	nh.advertise(chatter);
	while(1) {
		  str_msg.data = hello;
		  chatter.publish( &str_msg );
		  nh.spinOnce();
		  printf("chattered\n");
		  sleep(1);
	}
}
```



### The Code Explained

Now, let's break the code down. As a part of every ROS embedded linux
program, you need to include the `ros.h` header file and header files
for any messages that you will be using. Next, we need to instantiate
the node handle, which allows our program to create publishers and
subscribers. The node handle also takes care of communications over
serial port or LAN. We need to instantiate the publishers and
subscribers that we will be using. Here we instantiate a Publisher with
a topic name of "chatter". The second parameter to Publisher is a
reference to the message instance to be used for publishing. !RosSrvrIp
is the IP address of the ros workstation where rosserial_python runs;
edit this value and set it to the IP address of your ros workstation.
Rosserial_python will proxy the published messages to the rest of ROS.
Hello is the message to be published - a static string. In the main
function you need to initialize your ROS node handle. You need to pass
in the IP address (and port number if needed) of the system running
rosserial_python. If you don't pass in a parameter, the system will use
a default serial port.

!NodeHandle.initNode() accepts arguments like:

- /dev/ttyUSB1 uses the designated serial port for messages to
  rosserial_python
- 192.168.1.2 connects to rosserial_python at the specified IP address
  on the default port 11411
- 192.168.1.2:12345 connects to rosserial_python at the specified IP
  address on the specified port number Advertise any topics being
  published, and subscribe to any topics you wish to listen to.



```
while(1) {
		  str_msg.data = hello;
```

 Finally the program enters a while(1) loop in
which it publishes data then sleeps for a second, forever. We use the
string name as a pointer and assign it to the message data pointer.


```
chatter.publish( &str_msg );
		  nh.spinOnce();
		  sleep(1);
	}
}
```

 In the loop, the node publishes "Hello ROS"
and calls `ros::spinOnce()` where all of the ROS communication
callbacks are handled.

### Building and Uploading the Code

Use your system-specific procedures to build the program and load it
onto your embedded linux system. This may involve an IDE like TerkIDE,
or scp and running from a shell, or other procedures. Specific
instructions for your system may be documented in the Installation,
Build and Development Overview section of the rosserial_embeddedlinux
installation
[tutorial](#installing-rosserial_embeddedlinux)

### Testing the Code

Now, launch [roscore](roscore) on your ros workstation:


```
$ roscore
```



Next, in an new window on your ros workstation, run the
rosserial_python.py proxy application that passes your embedded linux
ROS messages to and from the rest of ROS. Make sure to specify tcp as
the communication channel: 

```
rosrun rosserial_python serial_node.py tcp
```

 The tcp argument
causes serial_node.py to become a server and wait for a connection from
a client. serial_node.py prints a message saying it's waiting for a
connection (from your embedded linux system).

As noted above, you can alternately specify a serial port on your ros
workstation that serial_node.py should use, provided you also change the
helloros application to specify a serial port on the embedded linux
system to use.

Now start the helloros application on your embedded linux system:


```
# cd <download_directory>
# ./helloros
```

 The helloros application prints "chattered"
each time it publishes a message.

Finally, watch the greetings come in from your embedded linux system by
launching a new terminal window on your ros workstation and entering :


```
rostopic echo chatter
```



Once you've started the helloros program it will attempt to establish
communication with the rosserial_python node and publish its message,
over and over, ad infinitum, ad nauseum.

### Restarting the embedded system or program

The helloros program can be stopped and restarted at will, and it will
reconnect to rosserial_python. Indeed, the embedded linux system can be
restarted at will, and the next time you run helloros it will reconnect
to the rosserial_python node.

Start then terminate the helloros program. Note that rostopic stops
printing - messages are no longer being published. Note that
rosserial_python prints an error message - it detected the loss of
communication with the embedded system.



```
[INFO] [WallTime: 1348628129.052204] socket.error exception caught
```



Now re-run helloros. Note that rosserial_python detects a new
connection, and rostopic echo starts printing the published messages
again.



```
[INFO] [WallTime: 1348628319.232685] Established a socket connection from 192.168.11.8 on port 40260
[INFO] [WallTime: 1348628319.232919] Forking a socket server process
[INFO] [WallTime: 1348628319.235348] launched startSocketServer
waiting for socket connection
[INFO] [WallTime: 1348628319.237011] starting ROS Serial Python Node serial_node-40260
[INFO] [WallTime: 1348628322.310832] Note: publish buffer size is 512 bytes
[INFO] [WallTime: 1348628322.311974] Setup publisher on chatter [std_msgs/String]
```

## Hello Embedded Linux (example subscriber)

- **Note:** This tutorial assumes that you have completed the previous
  tutorials:
  [Hello Ros (example publisher)](#hello-ros-example-publisher).
- **Description:** This tutorial shows how to make an embedded linux
  application subscribe to a topic published in ROS.
- **Tutorial Level:** BEGINNER
- **Next Tutorial:**
  [Example service](#example-service)

We'll continue our exploration into rosserial for embedded linux by
creating a subscriber program for our embedded linux box. It will
subscribe to the same topic as in the previous tutorial (chatter), and
print strings that are published to the chatter topic.

### The Code

Use your favorite editor or IDE to open the !ExampleSubscriber tutorial
code which is found in examples/ExampleSubscriber/ExampleSubscriber.cpp.
You should see the following:



```cplusplus
/*
 * rosserial_embeddedlinux subscriber example
 *
 * Prints a string sent on a subscribed ros topic.
 * The string can be sent with e.g.
 * $ rostopic pub chatter std_msgs/String -- "Hello Embedded Linux"
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <stdio.h>

ros::NodeHandle  nh;
char *rosSrvrIp = "192.168.15.149";

void messageCb(const std_msgs::String& received_msg){
	printf("Received subscribed chatter message: %s\n", received_msg.data);
}
ros::Subscriber<std_msgs::String> sub("chatter", messageCb );

int main()
{
	//nh.initNode();
	nh.initNode(rosSrvrIp);
	nh.subscribe(sub);

	while(1) {
		  sleep(1);
		  nh.spinOnce();
	}
}
```



### The Code Explained

Now, let's break the code down. The beginning of the program is very
similar to that of the Hello Ros example publisher in the previous
tutorial. As before, you need to include the `ros.h` header file and
header files for any messages that you will be using; in this case, the
String message.

You need to declare the node handle which will be used to create the
subscriber and handle communications to rosserial_python. !RosSrvrIp is
the IP address of the ros workstation where rosserial_python runs; edit
this value and set it to the IP address of your ros workstation.
Rosserial_python will proxy the subscribed messages from the rest of
ROS, and even from your embedded linux system! You then create the
callback function for our subscriber. The call back function will be
called whenever a message on topic "chatter" is received. It takes a
constant reference of a message as its argument. In our callback
messageCb, the type of message is std_msgs::String and the message name
will be received_msg. We need to instantiate the publishers and
subscribers that we will be using. Here we instantiate a subscriber with
a topic name of "chatter" and type std_msgs::String. Instantiating the
subscriber connects the topic to the callback function.

With subscribers, you must remember to template the subscriber upon the
message. Its two arguments are the topic it will be subscribing to and
the callback function it will be using. As before, you need to
initialize your ROS node handle, and pass in the IP address (and port
number if needed) of the system running rosserial_python. Advertise any
topics being published, and subscribe to any topics you wish to listen
to. Finally the program enters a while(1) loop in which it sleeps for a
second then handles any communication callbacks. This will result in the
subscriber callback function being called when a message on the
subscribed topic is received.

### Building and Uploading the Code

As before, use your system-specific procedures to build the program and
load it onto your embedded linux system.

### Testing the Code

If you haven't already done so, launch [roscore](roscore) on your ros
workstation and start the rosserial_python.py proxy application that
passes your embedded linux ROS messages to and from the rest of ROS.
Make sure to specify tcp as the communication channel:


```
rosrun rosserial_python serial_node.py tcp
```

 If rosserial_python.py is still running from
the previous tutorial, you can leave it running. It handles rosserial
tcp connections and associated subscriptions and publications as they
come and go.

Now start the !ExampleSubscriber application on your embedded linux
system: 

```
# cd <download_directory>
# ./ExampleSubscriber
```

 The subscriber is now waiting for
messages on the chatter topic, and will print them as they arrive.

Open a new window on your ros workstation and run the command below,
which publishes the string "Hello Embedded Linux" to the chatter
topic: 

```
$ rostopic pub chatter std_msgs/String -1 -- "Hello Embedded Linux"
```



You should see "Hello Embedded Linux" printed to stdout from the
!ExampleSubscriber application. The -1 parameter tells rostopic to
publish just once then exit. The \-- causes whatever follows to be sent
to rostopic on stdin, with the result that it gets used as data for the
String message that's published to chatter.

If you know a better way to publish a string, please edit this wiki page
and add it.

Finally, let's run the publisher from the last tutorial as a separate
application. Leave the !ExampleSubscriber application running, and open
a new window on your embedded linux system. Start the helloRos
application from the previous tutorial. Each time helloRos prints
"chattered" you should see !ExampleSubscriber print "Received
subscribed chatter message: Hello Ros!".

You can monitor what's going on by running a couple of programs on your
ros workstation: 

```
$ rxgraph &
$ rostopic echo chatter
```

 This starts the ros node
display program in the background, and runs rostopic in the foreground.

You will see two serial_node_xxxx nodes in the rxgraph window. One, the
helloRos publisher proxy, is publishing /chatter to rostopic, and also
to the second serial_node_xxxx, which is the !ExampleSubscriber proxy.
xxxx is the process ID - each new rosserial_embeddedlinux connection
gets a new process to proxy for it.

**Warning:** The rosserial_embeddedlinux package is not designed to
handle continuous disconnects and reconnects - you'll run out of nodes.
Instead, it's designed to gracefully recover from the occasional
exception condition.

## Example service

- **Description:** This tutorial shows how to make an embedded linux
  application offer a service to ROS service clients
- **Tutorial Level:** BEGINNER
- **Next Tutorial:**
  [VEXPro servo position control example](#vexpro-servo-position-control-example)

We'll continue our exploration into rosserial for embedded linux by
creating a service program for our embeddded linux box.

The publish/subscribe model described in previous tutorials is good for
sending data around the system - it decouples data producers from data
consumers so that neither knows or cares if the other is active,
inactive, or restarting. It's well-suited to one-way data flow.
However, that model has limitations; there are situations when a client
wants to request something to be done and know whether it was successful
or what the result was. For example, a system startup node may want to
request another node to run diagnostics and return the result, or to
change mode or grab something and return whether the request was
successful. For those purposes, the traditional client-server model is
more appropriate. Rosserial allows the embedded linux box to act as
either a client or a server. This tutorial describes the example code
that acts as a server and offers a service to any clients that request
it.

### The code

Use your favorite editor or IDE to open the ExampleService.cpp tutorial
code which is found in examples/ExampleService.cpp. You should see the
following:



```cplusplus
/*
 * rosserial_embeddedlinux service server example
 *
 * Advertises a service it offers. Prints the string sent to the service
 * and responds with an alternating string. It also publishes "hello world" on /chatter,
 * just to show it's alive.
 * The service request can be sent from the ROS command line with e.g.
 * $ rosservice call /test_srv "Hullo mate!"
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <rosserial_embeddedlinux/Test.h>
#include <stdio.h>

ros::NodeHandle  nh;
using rosserial_embeddedlinux::Test;
#define ROSSRVR_IP "192.168.15.149"

int i=0;
void svcCallback(const Test::Request & req, Test::Response & res){
	if((i++)%2)
		res.output = "hello";
	else
		res.output = "ros";
	printf("Service request message: \"%s\" received, responding with: %s\n", req.input, res.output);
}
ros::ServiceServer<Test::Request, Test::Response> server("test_srv",&svcCallback);

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

int main()
{
	nh.initNode(ROSSRVR_IP);
	nh.advertiseService(server);
	nh.advertise(chatter);

	str_msg.data = hello;

	while(1) {
		chatter.publish(&str_msg);
		nh.spinOnce();
		sleep(1);
	}
}
```



### The code explained

Now let's break the code down. The program is very similar to that of
the subscriber in the previous tutorial example. The Test.h header file
is automatically generated from the service message definition in
rosserial_embeddedlinux/srv/Test.srv. More information on creating
service messages is found at
[rosserial messages](https://index.ros.org/p/rosserial) svcCallback is
the function called when a service request is received. It is passed a
reference to the incoming request message, and to an outgoing response
message which it must fill. It fills alternate outgoing response
messages with "hello" then "ros", for a little variety (but only a
little). Then it prints the incoming request message data. The Test
Request and Response messages simply carry a string.

After defining the callback function, you declare the name of the
server, the names of its request and response messages, the name of the
service offered, and the callback function name.

Subsequent lines declare a publisher, as was done in the example
publisher tutorial. As before, you need to initialize your node handle,
passing it the IP of the system running the rosserial_python proxy. Then
you have to advertise the service being offered (and the publisher we
instantiated too). Finally, the program enters a loop, continually
publishing "hello" and calling spinOnce() to give the interface a
chance to receive any service requests and call the callback function.

### Testing the code

As before, use your system-specific procedures to build the program and
load it onto your embedded linux system.

If you haven't already done so, launch roscore on your ros workstation
and start the rosserial_python.py proxy application that passes your
embedded linux ROS messages to and from the rest of ROS. You can specify
tcp or the name of a serial port connected to your embedded linux
system.



```
$ rosrun rosserial_python serial_node.py tcp
```



Now start the ExampleService application on your embedded linux system.



```
$ ./ExampleService
connected to server
EmbeddedHardware.h: opened comm port successfully
```



Finally, run the rosservice utility on the ROS workstation several times
to send several service requests to your embedded system:


```
$ rosservice call /test_srv "Hullo mate"
$ rosservice call /test_srv "Hullo mate"
...
```



Observe the ExampleService program printing a message each time it
receives a messsage:



```
Service request message: "Hullo mate" received, responding with: ros
Service request message: "Hullo mate" received, responding with: hello
Service request message: "Hullo mate" received, responding with: ros
Service request message: "Hullo mate" received, responding with: hello
```

## Installing rosserial_embeddedlinux

- **Note:** This tutorial assumes that you have completed the previous
  tutorials: [ROS Tutorials](ROS/Tutorials).
- **Description:** This tutorial shows how to set up the ROS
  environment, and gives some hints to consider when setting up your
  embedded linux development environment.
- **Tutorial Level:** BEGINNER
- **Next Tutorial:**
  [Create an Example Publisher](#hello-ros-example-publisher)

### Installation, Build and Development Overview

The list below is a summary of the steps you'll take to set up the
build environment for your embedded linux system. 1. Install the
rosserial stack on your ros workstation (the Ubuntu system where you
build and run ROS software). 1. Build the rosserial_embeddedlinux
package. This creates the libraries and examples directories. 1. Copy
the libraries and examples directories to your embedded linux
development environment (which may be on your ros workstation or
elsewhere: e.g. a Windows PC). 1. Configure your embedded linux
development environment by adding the libraries/ros_lib directory to the
#include search path 1. Build an embedded linux application that uses
ros interfaces (usually with a cross-compiler - see your development
environment documentation) 1. Download the application to the target
embedded linux system and test/debug.

Follow the specific installation instructions and tutorial build
instructions for your particular embedded linux system below if
provided. If your system is not listed, follow the instructions for the
Generic Embedded Linux System, and please consider enhancing this
documentation by writing a step-by-step instruction for your system.

- [Generic Embedded Linux System](#genericinstall)
- [VEXPro Terk IDE](#vexproinstall)
- Please add additional system install instructions here

### Using the Library in Embedded Linux ROS Nodes

Our ROS bindings are implemented as a set of files #included by your
program when you #include <ros.h>. Your program becomes a ROS node
when it runs.

In order to use the rosserial library in your own code, you must first
put 

```
#include <ros.h>
```

 prior to including any other header files,
e.g. 

```
#include <std_msgs/String.h>
```

 otherwise the compiler may not be able to
locate them.

Now build and run the
[Hello Ros (example publisher) tutorial](#hello-ros-example-publisher)

## VEXPro range motor control loop

- **Description:** This tutorial describes how to use interrupts to
  accurately measure range using an ultrasonic rangefinder, then goes on
  to apply that in a motor control system with the loop controller
  running in a ROS node.
- **Tutorial Level:** INTERMEDIATE

This tutorial takes you step by step from measuring range and
controlling a motor through an integrated closed-loop control system for
maintaining an object at a constant distance. No steering logic is
introduced, so it is only useful in one dimension.

Four programs are presented in this tutorial, and the code for three are
explained:

1\. VEXProRangePublish: a program which uses interrupts to accurately
measure the range of an object using an ultrasonic rangefinder, and
publishes the range. This program is explained in depth. 1.
VEXProMotor13Subscribe: a program which subscribes to a motor speed
topic and drives the VEXPro 2-wire motor output to the requested
speed. 1. VEXProRangeMotorLoop: a program which combines the first two
into a system that publishes the range and subscribes to motor speed
requests published by the fourth program. The differences between this
and VEXProRangePublish are explained in depth, showing how motor control
is integrated with rangefinding. 1. VEXProRangeMotorCtrlr: a ROS node
written in C++ which attempts to keep an object at a constant distance
by running a motor forward or backward as the object becomes nearer or
further away than the setpoint. With appropriate mechanics it can be the
controller part of a closed-loop control system. This program is
explained in depth.

### The code

Listings for three programs are presented here, along with explanations
of their operation. If you understand the first: VEXProRangePublish,
you'll easily see how motor control is integrated into the second:
VEXProRangeMotorLoop. The third is the C++ code for the ROS node which
does the closed-loop control. See "The code explained" section for
line-by-line explanations of each of these programs. The testing section
describes how to run all four programs and test them individually.

The code for all four programs is in the examples directory and you
should be able to open them in your Terk IDE.

#### VEXProRangePublish

This program simply publishes the range to an obstacle sensed by an
ultrasonic rangefinder.



```cplusplus
/*
 * VEXProRangePublish.cpp
 *
 *  Created on: Jul 12, 2012
 *      Author: bouchier
 *
 *  Publish the range from an ultrasonic ranging sensor connected to digital 1 (Input) &
 *  digital 2 (Output) on topic sonar1
 */

#include <ros.h>
#include <std_msgs/Int32.h>
#include <stdio.h>
#include <unistd.h>
#include "qegpioint.h"

ros::NodeHandle  nh;
std_msgs::Int32 range;
ros::Publisher sonar1("sonar1", &range);

char *rosSrvrIp = "192.168.11.9";

#define USPI 150
#define BIAS 300

unsigned long diff(struct timeval *ptv0, struct timeval *ptv1)
{
  long val;

  val = ptv1->tv_usec - ptv0->tv_usec;
  val += (ptv1->tv_sec - ptv0->tv_sec)*1000000;

  return val;
}

void callback(unsigned int io, struct timeval *ptv, void *userdata)
{
  static struct timeval tv0;
  static int flag = 0;
  int sonarVal;

   if (io==0)
     {
       flag = 1;
       tv0 = *ptv;
     }

  if (io==1 && flag)
    {
	  sonarVal = diff(&tv0, ptv);
	  if (sonarVal>BIAS)
		  sonarVal = (sonarVal-BIAS)/USPI;
	  range.data = sonarVal;
      printf("%d\n", sonarVal);
    }
}

// Note: connector labeled "INPUT" on sonar sensor goes to
// digital 1 (bit 0), and connector labeled "OUTPUT" goes to
// digital 2 (bit 1).
int main()
{
  CQEGpioInt &gpio = CQEGpioInt::GetRef();
  volatile unsigned int d;

  // reset bit 0, set as output for sonar trigger
  gpio.SetData(0x0000);
  gpio.SetDataDirection(0x0001);

  // set callbacks on negative edge for both bits 0 (trigger)
  // and 1 (echo)
  gpio.RegisterCallback(0, NULL, callback);
  gpio.RegisterCallback(1, NULL, callback);
  gpio.SetInterruptMode(0, QEG_INTERRUPT_NEGEDGE);
  gpio.SetInterruptMode(1, QEG_INTERRUPT_NEGEDGE);

	//nh.initNode();
	nh.initNode(rosSrvrIp);
	nh.advertise(sonar1);

  // trigger sonar by toggling bit 0
  while(1)
    {
      gpio.SetData(0x0001);
      for (d=0; d<120000; d++);
      gpio.SetData(0x0000);
      sleep(1);		// the interrupt breaks us out of this sleep
      sleep(1);		// now really sleep a second
	  sonar1.publish( &range );
	  nh.spinOnce();
    }
}
```



##### The code explained

Now let's break the code down. Here's how VEXProRangePublish works. In
addition to the std_msgs/Int32.h message used to publish the range, you
include the
[libqwerk definition of the GPIO with Interrupts library](http://content.vexrobotics.com/vexpro_api/classCQEGpioInt.html):
qegpioint.h. This gives control over the digital I/O pins on the VEXPro,
and enables assigning callbacks to their interrupt.

The declaration of the publisher on the sonar1 topic at line 19 is
similar to the Hello ROS example publisher. The diff() function takes
two linux timeval structs and subtracts them. Timeval presents time in
two values representing seconds and microseconds. The time difference is
return in microseconds. This function is used to calculate the time
difference between the emitted and returning ultrasonic pulse. The
callback() function is called at interrupt level on the negative edge of
the output and input pins of the rangefinder. This happens first when
the code in main() drives the output negative (the start of the acoustic
pulse), and a second time when the echo is received. On the first call,
it stores the current time in tv0. On the second call, it subtracts tv0
from the current time, divides it by twice the speed of sound in
microseconds per inch (USPI), with the result being inches to the
reflective surface. It prints the value and stores it in the data part
of the range message, which will later be published by main(); Here you
declare the GPIO object so you can use its methods to operate on the
digital pins of the VEXPro controller. main() sets up the digital I/O
pins so that digital port 1 (which corresponds to bit 0) is an output,
and both are low. It then registers callback() to be called when either
digital port 1 or 2 (bits 0 or 1) transition low. These trigger the
timing measurement in callback() described above.

The node initialization and publisher advertisement on lines 78 - 79
work as described in the Hello ROS tutorial. The program then loops
forever in the while(1) loop, only breaking out when you terminate
execution, e.g. with \^C or by stopping it with the IDE. First it sets
digital pin 1 high, spins a while while the rangefinder sets up, then
sets the pin low, which triggers the rangefinder to emit the ultrasonic
pulse, and also triggers the first time measurement by callback(). It
then sleeps. The interrupt that happens when the reflected ultrasonic
pulse returns and digital pin 2 (the input) goes low triggers the second
time measurement and range calculation by callback(), but also breaks
main() out of its sleep. The second sleep() statement on line 88 ensures
the program actually sleeps for a second between range measurements.
When the sleep ends, the program publishes the range, and calls
spinOnce() which sends the message to ROS (and checks for any other work
that needs doing).

#### VEXProRangeMotorLoop

This program integrates motor operation into the rangefinder program
above.



```cplusplus
/*
 * VEXProRangeMotorLoop.cpp
 *
 *  Created on: Jul 13, 2012
 *      Author: bouchier
 *  Publish the range from a Sonar connected to digital 1 (Input) &
 *  digital 2 (Output) on topic sonar1. Control motor 13 speed by
 *  publishing the desired speed on a ros topic with e.g.
 * $ rostopic pub my_topic std_msgs/Int32 120.
 * Drives the motor on VEXPro motor13 connection to the requested value: -255 to +255
 *  that is received on subscribed topic motor1
 *
 * Note: connector labeled "INPUT" on sonar sensor goes to
 * digital 1 (bit 0), and connector labeled "OUTPUT" goes to
 * digital 2 (bit 1).
 */


#include <ros.h>
#include <std_msgs/Int32.h>
#include <stdio.h>
#include <unistd.h>
#include <qegpioint.h>
#include <qemotoruser.h>

ros::NodeHandle  nh;
std_msgs::Int32 range;
ros::Publisher sonar1("sonar1", &range);
CQEMotorUser &motor = CQEMotorUser::GetRef();	// motor singleton
CQEGpioInt &gpio = CQEGpioInt::GetRef();		// GPIO singleton

char *rosSrvrIp = "192.168.11.9";

#define USPI 150
#define BIAS 300

/*
 * Motor callback - called when new motor speed is published
 */
void motorCb(const std_msgs::Int32& motor13_msg){
	int speed = motor13_msg.data;
	printf("Received subscribed motor speed %d\n", speed);
    motor.SetPWM(0, speed);
}
ros::Subscriber<std_msgs::Int32> motorSub("motor13", motorCb );

/*
 * Calculate difference in usec between sonar start & end timevals
 */
unsigned long diff(struct timeval *ptv0, struct timeval *ptv1)
{
	long val;

	val = ptv1->tv_usec - ptv0->tv_usec;
	val += (ptv1->tv_sec - ptv0->tv_sec)*1000000;

	return val;
}

/*
 * Sonar callback. Called at interrupt level when sonar output transitions,
 * indicating end of range measurement
 */
void callback(unsigned int io, struct timeval *ptv, void *userdata)
{
	static struct timeval tv0;
	static int flag = 0;
	int sonarVal;

	if (io==0)
	{
		flag = 1;
		tv0 = *ptv;
	}

	if (io==1 && flag)
	{
		sonarVal = diff(&tv0, ptv);
		if (sonarVal>BIAS)
			sonarVal = (sonarVal-BIAS)/USPI;
		range.data = sonarVal;
		//printf("%d\n", sonarVal);
	}
}

int main()
{
	volatile unsigned int d;

	/* initialize ROS & subscribers & publishers */
	//nh.initNode();
	nh.initNode(rosSrvrIp);
	nh.advertise(sonar1);	// advertise sonar range topic
	nh.subscribe(motorSub);		// subscribe to motor speed topic

	// reset bit 0, set as output for sonar trigger
	gpio.SetData(0x0000);
	gpio.SetDataDirection(0x0001);

	// set callbacks on negative edge for both bits 0 (trigger)
	// and 1 (echo)
	gpio.RegisterCallback(0, NULL, callback);
	gpio.RegisterCallback(1, NULL, callback);
	gpio.SetInterruptMode(0, QEG_INTERRUPT_NEGEDGE);
	gpio.SetInterruptMode(1, QEG_INTERRUPT_NEGEDGE);

	// trigger sonar by toggling bit 0
	while(1)
	{
		gpio.SetData(0x0001);
		for (d=0; d<120000; d++);
		gpio.SetData(0x0000);
		usleep(100000);		// the interrupt breaks us out of this sleep
		usleep(100000);		// now really sleep
		sonar1.publish( &range );
		nh.spinOnce();
	}
}
```



##### The code explained

The VEXProRangeMotorLoop program is only a small extension of the
VEXProRangePublish program, having the addition of a motor speed
subscriber. This section describes just the motor speed logic
extensions; refer to the VEXProRangePublish description for the rest of
the code. In fact, the motor speed subscriber uses the same logic
previously described in the
[Example Subscriber tutorial](#hello-embedded-linux-example-subscriber),
so we will only talk about how to control a motor using libqwerk. In
addition to the std_msgs/Int32.h message used to publish the requested
motor speed, you include the
[libqwerk motor control library](http://content.vexrobotics.com/vexpro_api/classCQEMotorUser.html)
declarations, which provide access to the speed control methods. You get
a reference to the motor control object, whose methods you will use. The
motor speed callback is called whenever a message on the subscribed
topic "motor13" is received. It takes the requested speed out of the
Int32 message data and calls the motor's SetPWM() method. SetPWM()
controls the pulse-width modulation of motor 13's H-bridge. The first
argument is the motor index: 0 is motor 13, the first 2-wire motor port
on the VEXPro. The valid range of speed values is -255 to +255,
corresponding to full backward to full forward; 0 is stopped. See the
[libqwerk motor control library](http://content.vexrobotics.com/vexpro_api/classCQEMotorUser.html)
for a full list of available methods and arguments.

After defining the callback, the code instantiates the subscriber,
telling it the topic name and callback function.

One other difference from VEXProRangePublish: this program sleeps in
main() for only 100ms, giving it a much faster publish rate, and making
the control loop faster.

That's it! Pretty straightforward. Next we'll look into the code that
runs under ROS, and that subscribes to the range messages and publishes
the motor speed messages to VEXProRangeMotorLoop.

#### VEXProRangeMotorCtrlr

This program runs as a ROS node on your ROS workstation and subscribes
to rangefinder readings. Depending on whether the range is closer or
further away than 20 inches, it runs the motor one way or the other -
faster the further from the setpoint of 20 inches.



```cplusplus
#include "ros/ros.h"
#include "std_msgs/Int32.h"

int range = 20;

void sonarCallback(const std_msgs::Int32::ConstPtr& sonarMsg)
{
  ROS_INFO("sonar range: [%d]", sonarMsg->data);
  range = sonarMsg->data;
}

int
main (int argc, char **argv)
{
  // inialize ros
  ros::init (argc, argv, "VEXProRangeMotorCtrlr");
  ros::NodeHandle n;
  ros::Rate loop_rate (2);
  std_msgs::Int32 speedMsg;
  std_msgs::Int32 positionMsg;

  // declare the publishers & subscribers
  ros::Publisher motor13_pub =
    n.advertise < std_msgs::Int32 > ("motor13", 1000);
  ros::Subscriber sonarSub = n.subscribe("sonar1", 1000, sonarCallback);

  int speed = 0;

  while (ros::ok ())
    {
      speed = std::min(range, 40);	// clamp range to a max
      speed = (speed * 10) - 200;		// give it a range of -200 - +200, with 0 at 20"
      speedMsg.data = speed;

      ROS_INFO ("range %d speed %d", range, speed);
      motor13_pub.publish (speedMsg);

      ros::spinOnce ();

      loop_rate.sleep ();
    }
  return 0;
}
```



##### The code explained

The VEXProRangeMotorCtrlr program runs on your ROS workstation. The
executable is built when you rosmake the rosserial_embeddedlinux package
on your ROS workstation, and is written into the bin directory. As
usual, you include the message types in use; in this case Int32 is used
for both range and motor speed. range is the current object distance; it
is written by the subscriber callback function and read by the main()
loop and motor control is based on this value. Just as on the VEXPro
side, the ROS side uses callbacks to notify a subscriber that a message
was received. sonarCallback() copies the range reading that was
published by the VEXProRangeMotorLoop program on VEXPro into the range
variable.

You can find an in-depth explanation of how this ROS subscriber code
works in [this tutorial](ROS/Tutorials/WritingPublisherSubscriber(c++)).
The node initializes its ROS subsystem. It gets a ROS Nodehandle, which
gives access to methods to subscribe, publish, etc. It sets its
loop-rate to 2Hz, and it declares the messages which will be used to
receive range and send motor speed requests. Next, the program
instantiates the motor publisher and range subscriber, passing them the
topic name and callback for the subscriber. Instantiating the publisher
and subscriber causes them to ask the ROS master for the location of the
node publishing or subscribing on those topics - this is the
rosserial_python.py node instance acting as a proxy for
VEXProRangeMotorLoop.

Note that the system supports multiple rosserial_python proxy nodes
servicing (usually) different topics. The program enters an endless
loop, in which range is converted to speed in lines 31-32 by limiting it
to 0 to 40 inches, then the distance range expanded to a speed range of
-200 to +200. 20 inches produces a speed of 0.

The program publishes the motor speed, calls spinOnce() to let messages
get sent, then sleeps for half a second in loop_rate.sleep() as long as
ROS is running and it hasn't been terminated.

### Testing the code

This section has three testing sections that use the programs described
in this tutorial.

#### Testing the rangefinder

Plug the input connector of the VEX ultrasonic rangefinder into digital
port 1, and the output connector into digital port 2. Like most
ultrasonic rangefinders, the Vex unit uses a trigger pulse to start the
ranging pulse, and emits a pulse when it detects the returning acoustic
reflection. The time between the falling edge of the trigger and the
falling edge of the output is the round-trip time of the acoustic wave,
from which the range to the reflective surface can be calculated.

As usual, make sure ros_core and serial_node.py are running. Build,
download, and start the VEXProRangePublish binary to the VEXPro
controller.

Use rostopic to echo the range measurement (in inches) that's published
by the VEXProRangePublish program once a second. Change the distance to
the reflective surface and watch the range change.



```
$ rostopic echo sonar1
data: 38
---
data: 38
---
data: 39
---
data: 15
---
data: 15
---
data: 14
---
```



#### Testing the motor

Plug a 2-wire motor into the Motor13 port on the VEXPro. Note: Vex
motors have plugs with male pins. You can adapt a 3rd-party motor by
using a servo extender cable and some solder - just remove the sheath
around the pins on the servo extender cable. The 2-wire motor connects
to the two pins furthest from the edge of the controller (corresponding
to the control and +5V pins of a servo or 3-wire motor). The controller
pulse-width-modulates battery voltage to the motor pins through an
H-bridge protected by a self-resetting fuse.

Build, download, and start the VEXProMotor13Subscribe binary to the
VEXPro controller. Use rostopic to publish the desired motor speed to
the listening subscriber. Note the use of \-- preceding a negative
speed. The valid speed range is -255 to +255.



```
$ rostopic pub -1 motor13 std_msgs/Int32 100
publishing and latching message for 3.0 seconds
$ rostopic pub -1 motor13 std_msgs/Int32 -- -100
publishing and latching message for 3.0 seconds
$ rostopic pub -1 motor13 std_msgs/Int32 0
publishing and latching message for 3.0 seconds
```



The program running on the VEXPro prints each speed as it receives it:



```
root@qwerk:~# /opt/usr/bin/VEXProMotor13Subscribe
Connecting to TCP server at 192.168.11.9:11411....
connected to server
Received subscribed motor speed 100
Received subscribed motor speed -100
Received subscribed motor speed 0
```



#### Pulling it all together: responsive control

This test builds on the two previous tests by running 1.
VEXProRangeMotorLoop on the VEXPro, which publishes sonar readings and
subscribes to the motor speed topic 1. VEXProRangeMotorCtrlr on the ROS
workstation, which reads the sonar topic and responds by publishing
motor speed/direction requests.

Build, download, and start the VEXProRangeMotorLoop binary to the VEXPro
controller. It will sit publishing range and waiting for the first motor
speed message.



```
root@qwerk:~# /opt/usr/bin/VEXProMotorRangeLoop
Connecting to TCP server at 192.168.11.9:11411....
connected to server
```



The VEXProMotorRangeCtrlr program was built when you build the embedded
linux package, and the binary is in the rosserial_embeddedlinux/bin
directory. Run it on your ROS workstation. The motor connected to the
VEXPro will start running. Move a reflective surface nearer or further
from the rangefinder and watch the motor speed and direction change.



```
$ rosrun rosseal_embeddedlinux VEXProRangeMotorCtrlr 
[ INFO] [1348455399.015726663]: range 20 speed 0
[ INFO] [1348455399.501250299]: range 20 speed 0
[ INFO] [1348455399.501761423]: sonar range: [28]
[ INFO] [1348455399.502253554]: sonar range: [28]
[ INFO] [1348455400.001241799]: range 28 speed 80
[ INFO] [1348455400.001640643]: sonar range: [28]
[ INFO] [1348455400.001790349]: sonar range: [28]
[ INFO] [1348455400.002211817]: sonar range: [28]
[ INFO] [1348455400.002438890]: sonar range: [28]
[ INFO] [1348455402.001361698]: range 27 speed 70
[ INFO] [1348455402.002070288]: sonar range: [26]
[ INFO] [1348455402.002388693]: sonar range: [25]
[ INFO] [1348455402.002700115]: sonar range: [23]
[ INFO] [1348455402.002869652]: sonar range: [22]
[ INFO] [1348455402.501255444]: range 22 speed 20
[ INFO] [1348455402.501669090]: sonar range: [21]
[ INFO] [1348455402.501814328]: sonar range: [21]
[ INFO] [1348455402.502216803]: sonar range: [20]
[ INFO] [1348455402.502413712]: sonar range: [19]
[ INFO] [1348455403.001236051]: range 19 speed -10
[ INFO] [1348455403.001656959]: sonar range: [19]
[ INFO] [1348455403.001812251]: sonar range: [18]
[ INFO] [1348455403.003050679]: sonar range: [18]
[ INFO] [1348455403.003198150]: sonar range: [17]
[ INFO] [1348455403.501292350]: range 17 speed -30
[ INFO] [1348455403.501671922]: sonar range: [17]
[ INFO] [1348455403.502039764]: sonar range: [17]
[ INFO] [1348455403.502313201]: sonar range: [16]
[ INFO] [1348455403.502652833]: sonar range: [16]
[ INFO] [1348455404.001357309]: range 16 speed -40
[ INFO] [1348455404.001723474]: sonar range: [16]
[ INFO] [1348455404.002102488]: sonar range: [15]
[ INFO] [1348455404.002332354]: sonar range: [15]
[ INFO] [1348455404.002524794]: sonar range: [15]
[ INFO] [1348455404.501225078]: range 15 speed -50
```



VEXProRangeMotorLoop running on the VEXPro prints the requested motor
speed:



```
Received subscribed motor speed 0
Received subscribed motor speed 80
Received subscribed motor speed 70
Received subscribed motor speed 20
Received subscribed motor speed -10
Received subscribed motor speed -30
Received subscribed motor speed -40
Received subscribed motor speed -50
```

## VEXPro servo position control example

- **Description:** This tutorial shows how to use the VEXPro embedded
  linux system with the libqwerk library that exposes device I/O to
  applications, in order to control the position of a servo.
- **Tutorial Level:** BEGINNER
- **Next Tutorial:**
  [VEXPro range motor control loop](#vexpro-range-motor-control-loop)

The ultimate goal of an embedded linux system is typically to control
robot actuators and to read sensors or cameras, and to pass robot data
between the actuators and sensors and the ROS nodes that perform
higher-level functions. Embedded linux systems typically have libraries
which facilitate accessing connected peripherals. The VEXPro controller
offers the
[libqwerk](http://content.vexrobotics.com/vexpro_api/annotated.html)
library for this purpose. Libqwerk provides a C++ interface to Vex
Robotics peripherals including motors, ultrasonic range sensors, servos
and more. The VEXPro is a general purpose controller and can also be
used with 3rd-party peripherals, provided proper understanding of the
interfaces is applied.

This tutorial shows how to integrate libqwerk control of a servo with a
ROS subscription, such that ROS nodes can publish the desired servo
position.

You should have already completed the rosserial and VEXPro software
installation described
[here](#vexproinstall), which
includes
[installing the VEXPro Terk IDE](http://code.google.com/p/terk-ide/wiki/InstallationInstructions)
and running the
[quickstart Terk tutorial](http://code.google.com/p/terk-ide/wiki/QuickIDEExample).
If you haven't, you won't be able to run this tutorial (but reading
through it may still be interesting).

### The Code

Use your favorite editor or IDE to open the VEXProServoSubscribe.cpp
tutorial code which is found in examples/VEXProServoSubscribe.cpp. You
should see the following: 

```cplusplus
/*
 * VEXProServoSubscribe.cpp
 *
 *  Created on: Jul 12, 2012
 *      Author: bouchier
 *  Drives a servo or motor on VEXPro motor1 connection to the requested value: 0 - 255
 *  that is received on subscribed topic servo1
 */

#include <ros.h>
#include <std_msgs/Int32.h>
#include <iostream>
#include <qeservo.h>
using namespace std;

ros::NodeHandle  nh;
CQEServo &servo = CQEServo::GetRef();
char *rosSrvrIp = "192.168.11.9";

void messageCb(const std_msgs::Int32& servo1_msg){
	int position = servo1_msg.data;
	printf("Received subscribed servo position %d\n", position);
	servo.SetCommand(0, position);
}
ros::Subscriber<std_msgs::Int32> sub("servo1", messageCb );

int main() {
	//nh.initNode();
	nh.initNode(rosSrvrIp);
	nh.subscribe(sub);

	while(1) {
		  sleep(1);
		  nh.spinOnce();
	}
}
```



### The code explained

Now let's break the code down. You include <std_msgs/Int32.h> as it
is the message type that will be published with the desired position in
its low-order byte. (The code can be easily extended to address
different servos by encoding them in an upper byte.)

You include <iostream> to declare the printf() function. Include
<qeservo.h> to get the libqwerk servo control class. See the
[libqwerk servo class documentation](http://content.vexrobotics.com/vexpro_api/classCQEServo.html)
for full details on all the methods and members related to controlling
servos. A unique feature of libqwerk is that the application obtains a
reference to the hardware control object (of which there is only one,
and only one reference can be acquired). This reference is to the object
which provides the hardware control methods. In this case, we get a
reference to the servo control object. Define the callback function
that's invoked when a ROS node publishes to the servo1 topic. The
function pulls the desired position out of the message and uses the
CQEServo class' SetCommand() method to set the servo position. The
first argument is the servo number; servo 0 corresponds to motor 1.
Declare the subscriber, as was done in the example subscriber of a
previous tutorial, and pass it the name of the topic and the callback
function.

#### main()

The rest of the program is the same as in the
[example subscriber](https://index.ros.org/p/rosserial_embeddedlinux)
tutorial: you initialize the node handle passing it the IP address of
the rosserial_python proxy, subscribe, and then periodically run the
spinOnce() function which polls the interface for incoming messages.

### Testing the code

Plug a servo into the Motor1 port on the VEXPro. Note: Vex servos have
plugs with male pins, whereas typical hobby servos have female pins. You
can adapt one to the other with a 3-pin section of straight male header
pins and an optional servo extender cable with the sheath around the
pins removed.

As usual, make sure ros_core and serial_node.py are running. Build,
download, and start the VEXProServoSubscribe binary on your embedded
linux system.



```
root@qwerk:/opt/usr/bin# ./VEXProServoSubscribe 
Connecting to TCP server at 192.168.11.9:11411....
connected to server
EmbeddedHardware.h: opened comm port successfully
```



Each time an embedded linux client connects using TCP/IP,
rosserial_python creates a new socket server process, and creates a new
node with a name that includes the port number being used for that
connection. This is logged by the rosserial_python proxy.

Notice also that the subscription to the servo1 topic is logged by the
rosserial_python proxy.



```
[INFO] [WallTime: 1348389339.177539] Established a socket connection from 192.168.11.8 on port 54788
[INFO] [WallTime: 1348389339.177657] Forking a socket server process
[INFO] [WallTime: 1348389339.181057] launched startSocketServer
waiting for socket connection
[INFO] [WallTime: 1348389339.183572] starting ROS Serial Python Node serial_node-54788
[INFO] [WallTime: 1348389342.239358] Note: subscribe buffer size is 512 bytes
[INFO] [WallTime: 1348389342.240773] Setup subscriber on servo1 [std_msgs/Int32]
```



Now publish servo values from 0 - 255 to topic servo1 and watch the
servo move to the requested position.



```
$ rostopic pub -1 servo1 std_msgs/Int32 20
publishing and latching message for 3.0 seconds
$ rostopic pub -1 servo1 std_msgs/Int32 200
publishing and latching message for 3.0 seconds
```

## GenericInstall

### Installing Rosserial from Source

At present, installing from source is the only way to get the
rosserial_embeddedlinux package. You'll need to do these few steps.

**Note:** source build instructions are different for groovy (catkin)
than for earlier (rosbuild) releases. Select the build system based on
your release to see appropriate instructions. 

```
{#!wiki buildsystem catkin

Rosserial has been catkin-ized, and the workflow is a bit different from in the past. Rather than running the library generator over each package you want to use, you run it once and generate libraries for all installed messages. In the instructions below, <ws> represents your catkin workspace. 


{{{
  cd <ws>/src
  git clone https://github.com/ros-drivers/rosserial.git
  cd <ws>
  catkin_make
  catkin_make install
  source <ws>/install/setup.bash
```



These commands clone rosserial from the github repository, generate the
rosserial_msgs needed for communication, and make the `ros_lib`
library in the <ws>/install directory.

Note: currently you HAVE to run catkin_make install, otherwise portions
of the ros_lib directory will be missing. This will hopefully be fixed
soon.

}}}}



```
{#!wiki buildsystem rosbuild

{{{
hg clone https://kforge.ros.org/rosserial/hg rosserial
rosmake rosserial_embeddedlinux
```



These commands clone rosserial from the kforge repository using
mercurial, generate the rosserial_msgs needed for communication, and
make the `ros_lib` library.

}}}}

### Install rosserial_embeddedlinux into your Development Environment

Ros_lib installation instructions are different for groovy source
(catkin) than for earlier (rosbuild) releases. Be sure you've selected
the correct build system above to see appropriate instructions - catkin
for a groovy source build, rosbuild otherwise.



```
{#!wiki buildsystem catkin

You need to create the ros_lib and examples folders in some appropriate directory. These commands also copy the examples directory to the target location. 

{{{
  cd <some_directory>
  rm -rf ros_lib examples
  rosrun rosserial_embeddedlinux make_libraries.py .
```



**Note:** you have to delete ros_lib in order to regenerate as its
existence causes an error.

}}}}



```
{#!wiki buildsystem rosbuild

Copy the `rosserial_embeddedlinux/libraries` directory into an appropriate place in your embedded linux development environment.
```

}

If necessary, copy the ros_lib and examples directories to a Windows
machine, or wherever you do your embedded linux development.

Now you've copied ros_lib and examples directories to an appropriate
place in your embedded linux development environment, try the embedded
linux [tutorials](#tutorials-1).

### Development Environment

Each embedded Linux system is typically supported by its own development
environment. Most cross-compile the target binary on a development host
such as a Windows or Linux PC or virtual machine. Refer to your
system's documentation for instructions on how to install the
development environment and how to deploy the built binary onto the
target.

### Building the Examples; Download, Test and Debug

The rosserial_embeddedlinux/examples directory contains a set of
examples that are described in these
[tutorials](#tutorials-1). You
compile them for your target, ensuring that the compiler's set of
include directories includes a path to the location you installed
*libraries/ros_lib*. This is typically done in a Makefile with a
-I<path_to_ros_lib> flag, or in a GUI with an include directory
project property.

The output of building the examples is an executable file built to run
on your embedded linux target system. Use your system-specific
procedures to copy the file to the target, run and debug it.

## VEXProInstall

### Install the Terk IDE

Install the development environment for the VEXPro controller - known as
the Terk IDE - on your windows PC or virtual machine following these
[installation instructions](http://code.google.com/p/terk-ide/wiki/InstallationInstructions).
We will call this system the VEXPro build system. (Ignore the
instructions for installing on Linux.)

Do the VEXPro Terk IDE example tutorial
[here](http://code.google.com/p/terk-ide/wiki/QuickIDEExample), to make
sure you can successfully build and run programs on the VEXPro
controller.

### Installing Rosserial from Source

At present, installing from source is the only way to get the
rosserial_embeddedlinux package. You'll need to do these few steps.

**Note:** source build instructions are different for groovy (catkin)
than for earlier (rosbuild) releases. Select the build system based on
your release to see appropriate instructions. 

```
{#!wiki buildsystem catkin

Rosserial has been catkin-ized, and the workflow is a bit different from in the past. Rather than running the library generator over each package you want to use, you run it once and generate libraries for all installed messages. In the instructions below, <ws> represents your catkin workspace. 


{{{
  cd <ws>/src
  git clone https://github.com/ros-drivers/rosserial.git
  cd <ws>
  catkin_make
  catkin_make install
  source <ws>/install/setup.bash
```



These commands clone rosserial from the github repository, generate the
rosserial_msgs needed for communication, and make the `ros_lib`
library in the <ws>/install directory.

Note: currently you HAVE to run catkin_make install, otherwise portions
of the ros_lib directory will be missing. This will hopefully be fixed
soon.

}}}}



```
{#!wiki buildsystem rosbuild

{{{
hg clone https://kforge.ros.org/rosserial/hg rosserial
rosmake rosserial_embeddedlinux
```



These commands clone rosserial from the kforge repository using
mercurial, generate the rosserial_msgs needed for communication, and
make the `ros_lib` library.

}}}}

### Install rosserial_embeddedlinux into the VEXPro Terk Development Environment

Ros_lib installation instructions are different for groovy source
(catkin) than for earlier (rosbuild) releases. Be sure you've selected
the correct build system above to see appropriate instructions - catkin
for a groovy source build, rosbuild otherwise.



```
{#!wiki buildsystem catkin

You need to create the ros_lib and examples folders in some appropriate directory. These commands also copy the examples directory to the target location. 

{{{
  cd <some_directory>
  rm -rf ros_lib examples
  rosrun rosserial_embeddedlinux make_libraries.py .
```



**Note:** you have to delete ros_lib in order to regenerate as its
existence causes an error.

Now create a tar archive of the libraries and examples directories in
the rosserial_embeddedlinux package as follows:



```
$ tar -cf rosserial_embeddedlinux.tar libraries examples
```



Transfer the .tar file to the Terk build system and untar it into a
suitable folder, such as C:\\Users\\**<username>**\\workspace.
(Replace <username> with your Windows username.) As part of the Terk
IDE install, you installed cygwin. Open a cygwin window and cd to the
workspace directory and untar it:



```
# cd /cygdrive/c/Users/<username>/workspace
# tar -xf rosserial_embeddedlinux.tar
```



You now have two new directories in your workspace: libraries, and
examples.

}}}}



```
{#!wiki buildsystem rosbuild

On your ROS workstation, create a tar archive of the libraries and examples directories in the rosserial_embeddedlinux package as follows:

{{{
$ roscd rosserial_embeddedlinux
$ tar -cf rosserial_embeddedlinux.tar libraries examples
```



Transfer the .tar file to the Terk build system and untar it into a
suitable folder, such as C:\\Users\\**<username>**\\workspace.
(Replace <username> with your Windows username.) As part of the Terk
IDE install, you installed cygwin. Open a cygwin window and cd to the
workspace directory and untar it:



```
# cd /cygdrive/c/Users/<username>/workspace
# tar -xf rosserial_embeddedlinux.tar
```



You now have two new directories in your workspace: libraries, and
examples.

}}}}

Now you've copied ros_lib and examples directories to an appropriate
place in your VEXPro development environment, try the embedded linux
[tutorials](#tutorials-1).

### Build the Second Tutorial

- Run the Terk IDE and press the "Create New Project" button
- In the popup dialog, type the new Project Name: "Hello Ros". (Do not
  type the quotes.) Uncheck "Use default location" and browse to the
  examples/HelloRos directory. Choose Project Type: "Empty Project"
  and Toolchain: "TerkOS Cygwin GCC". Finish the dialog and ignore the
  build failure.
- Right-click the Hello Ros project in the Project Explorer pane and
  select properties
- In the left navigation pane, expand the "C/C++ Build" property group
  by clicking the arrow, and select "Settings".
- In the main pane, "Tool Settings" tab, expand the "TerkOS C++
  Compiler" settings and select the "Directories" folder.
- Next to "Include Paths -I" click the + button and type the following
  path into the dialog:



```
../../../libraries/ros_lib
```



This gives the compiler the path to ros.h and the other library files
for rosserial_embeddedlinux. Adjust appropriately if you placed the
libraries elsewhere.

- Close the properties dialog with "Ok". From here, the build
  instructions are the same as for any Terk project:
- Select the console tab in the bottom half of the Terk IDE. This will
  display the output as the project builds
- Right-click the Hello Ros project in the project explorer and select
  "Clean Project". You should see that it "Finished building target"
  with no errors.
- If there were build errors, correct them.

Congratulations! You have now completed the first tutorial -
VEXProInstall. You are now ready to run and test it by following the
[Example Publisher tutorial](http://ros.org/wiki/rosserial_embeddedlinux/Tutorials/Example Publisher)
