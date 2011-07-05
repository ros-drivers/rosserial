/*
* ServoControl.pde 
*
* This sketch demonstrates the control of hobby R/C servos
* using ROS and the arduiono
* 
* For the full tutorial write up, visit
* www.ros.org/wiki/rosserial_arduino_tutorials
*
* For more information on the Arduino Servo Library
* Checkout :
* http://www.arduino.cc/en/Reference/Servo
*/

#include <WProgram.h>
#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>

Servo servo;



ROS_CALLBACK( servo_cb, std_msgs::UInt16, cmd_msg)
  
  servo.write(cmd_msg.data); //set servo angle, should be from 0-180
  
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}

ros::NodeHandle nh;
ros::Subscriber sub("servo", &cmd_msg, servo_cb);


void setup(){
 
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  
  
  servo.attach(0); //attach it to pin 0
  
}

void loop(){
  nh.spinOnce();
  delay(1);
}
