## Overview

The rosserial_server package contains a
C++ implementation of the host-side rosserial connection. It
automatically handles setup, publishing, and subscribing for a connected
rosserial-enabled device. It uses a python node in the rosserial_python
package as an aid for handling subcriptions.

These nodes use the
[topic_tools](https://index.ros.org/p/topic_tools)!/ShapeShifter meta
message in order to republish messages from clients without having to
know about them at compile time. The only caveat to this approach is
that the servers do not have the full textual definition of messages (as
it is not stored in clients or part of the rosserial protocol). There is
a provided Python shim which the servers may optionally use to acquire
this information. Example launch files for serial and socket usage are
provided in the package.

Common codebase supports Indigo, Hydro, and Groovy clients.

## Usage

Recommended usage is to launch the main server node alongside the
message info node. A launchfile is provided for starting the serial
link. If the port is not specified it defaults to /dev/ttyACM0.



```
roslaunch rosserial_server serial.launch port:=/dev/ttyUSB0
```

 There's also a launchfile for starting a
socket link, which defaults to listening on port 11411.


```
roslaunch rosserial_server socket.launch
```



Basic usage with just the server alone:



```
rosrun rosserial_server serial_node _port:=/dev/ttyUSB0
```



## Nodes

### serial_node

Interface to a single rosserial client over a serial connection. This
node attempts to connect and handshake on a 1Hz basis, and automatically
reconnected upon replug or other failure. Upon connection, it spins up
subscribers and publishers according to the configuration information
stored in the device.

#### Parameters

- `~port` (`str`, default: `/dev/ttyACM0`): Name of port to use.
- `~baud` (`int`, default: `57600`): Baud rate, in bps.
- `~require` (`obj`): See below

### socket_node

Interface to a multiple rosserial TCP clients.

#### Parameters

- `~port` (`int`, default: `11411`): Number of port to create TCP server
  on.

## Require API

The `require` parameter to the server allows it to enforce that the
client(s) must publish and subscribe to a certain set of topics. This is
important because a serial connection provides no guarantee of delivery,
and the rosserial protocol doesn't inherently contain any such checks.

Example usage in a launch file:



```
<launch>
  <node pkg="rosserial_server" type="serial_node" name="rosserial_server">
    <rosparam>
      port: /dev/arduino
      require:
        publishers: [ status ]
        subscribers: [ cmd, lights ]
    </rosparam>
  </node>
  <node pkg="rosserial_python" type="message_info_service.py"
        name="rosserial_message_info" />
</launch>
```



When the Arduino client connects to this server, the server will verify
that the status, cmd, and lights topics get set up. If they don't after
1 second, the server will re-initialize the connection.

Use of the `require` parameter is completely optional, but may be of
value to users needing to deploy ROS-connected microcontrollers into
environments where the occasional failed initialization is not
acceptable.

## Limitations

At this time, rosserial_server is
experimental. It is missing key features of the
[rosserial_python](https://index.ros.org/p/rosserial_python)-provided
node, including parameters, logging, and services. If you require these
features, please stick to the standard Python server for now.

Or be prepared to implement them yourself, which we'd really
appreciate!
