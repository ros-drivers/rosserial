/**
 * rosserial Publisher Example with ArduinoBluetoothHardware
 * Prints "hello world!"
 * This intends to be connected from bluetooth interface of host PC.
 * After pairing and trusting this device from PC, bind your device to serial device.
 *   sudo rfcomm bind 1 <MAC Address of your device>
 *   sudo stty -F /dev/rfcomm1 57600 cs8
 * then you can now connect rosserial host node to serial device
 *   rosrun rosserial_python serial_node.py _port:=/dev/rfcomm1 _baud:=57600
 */

#define ROSSERIAL_ARDUINO_BLUETOOTH

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[14] = "hello world!";

void setup()
{
    nh.initNode("BluetoothHelloworld");
    nh.advertise(chatter);

    // wait for bluetooth and rosserial host connection.
    while (not nh.connected()) {
        nh.spinOnce();
        delay(100);
    }
}

void loop()
{
    str_msg.data = hello;
    chatter.publish( &str_msg );
    nh.spinOnce();
    delay(1000);
}
