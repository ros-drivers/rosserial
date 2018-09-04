/*
 * Rosserial Vex Cortex keyboard/phone driving demo
 * 
 * For keyboard
 * ---------
 * install the keyboard twist publisher: http://wiki.ros.org/teleop_twist_keyboard
 *
 * run the command described in the link, and key control will work on the robot!
 * press center key to re-center.
 *
 * For phone
 * --------
 * Install ROS Nav Map from the google play store
 * follow setup instructions, use the joypad!
 */

#include <ros.h>
#include "geometry_msgs/Twist.h"

inline void handleControl(const geometry_msgs::Twist& t){
  // power motors using the message event!
  // (default configuration is for a clawbot set up in the standard fashion).
  motorSet(1, -(int) (100 * t.linear.x + 50 * t.angular.z));
  motorSet(10, (int) (100 * t.linear.x - 50 * t.angular.z));
}

// called from opcontrol.cpp
inline void setup()
{
  // debug logging
  vexroslog("\n\n\n\n\r\t\tROSserial for VEX Cortex V2 - June 2018 - TWIST\n\n\n\n\n\r");

  // make a node handle object and a subscriber for the Twist message.
  ros::NodeHandle  nh;
  ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel\0", &handleControl);

  // set up rosserial, and prepare to publish the chatter message 
  nh.initNode();
  nh.subscribe(sub);

  // message data variable.
  while (1) {

    // send a message about the time!
    nh.spinOnce();
    delay(20);
  }
}
