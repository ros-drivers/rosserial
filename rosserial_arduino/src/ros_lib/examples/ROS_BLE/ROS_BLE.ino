/*
 * ROS + RFDuino Example
 *
 * Author: Ilia Baranov
 * Subscribes to Byte data from ROS
 * Publishes string from received BLE data
 * 
 * To compile, download latest RFDuino Zip, unzip entire thing
 * into <arduino install folder>/hardware/arduino
 * Ensure ROS works with Arduino.
 * Suggest using the RFDuino Android/Iphone example to send/receive data.
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Byte.h>
#include <RFduinoBLE.h>
ros::NodeHandle  nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

void messageCb( const std_msgs::Byte& Smsg){
  RFduinoBLE.sendByte(Smsg.data);
  
}

ros::Subscriber<std_msgs::Byte> sub("test", messageCb );

char hello[15] = "              ";

void setup()
{
  override_uart_limit = true;
  Serial.begin(57600);
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
  RFduinoBLE.advertisementInterval = 100;
  RFduinoBLE.advertisementData = "ROS";
  RFduinoBLE.begin();
}

void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
}

void RFduinoBLE_onReceive(char *data, int len){
  for (int i=0; i<=len; i++)
  {
    hello[i] = data[i];
  }
}
