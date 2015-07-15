//#define COMPILE_SERVOCONTROL_CODE_ROSSERIAL
#ifdef  COMPILE_SERVOCONTROL_CODE_ROSSERIAL

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

#include "mbed.h"
#include "Servo.h"
#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh;

Servo servo(p21);
DigitalOut myled(LED1);

void servo_cb( const std_msgs::UInt16& cmd_msg) {
    servo.position(cmd_msg.data); //set servo angle, should be from 0-180
    myled = !myled;  //toggle led
}


ros::Subscriber<std_msgs::UInt16> sub("servo", servo_cb);

int main() {

    nh.initNode();
    nh.subscribe(sub);

    while (1) {
        nh.spinOnce();
        wait_ms(1);
    }
}
#endif