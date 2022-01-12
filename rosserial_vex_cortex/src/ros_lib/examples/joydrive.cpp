/*
 * Rosserial Vex Cortex joystick control demo
 * 
 * This lets you control a VEX Clawbot (or any other robot, with some modification)
 * using a Logitech Wireless gamepad F710.
 * 
 * install the joystick_drivers ROS system dependency: http://wiki.ros.org/joystick_drivers?distro=melodic
 * then run `rosrun joy joy_node _autorepeat_rate:=20`
 *
 * This also can work with a PS3 controller / wiimote (see the ROS joystick_drivers metapackage).
 * 
 * Or any other ROS node that publishes joystick events!
 */

#include "ros.h"
#include "main.h"
#include "sensor_msgs/Joy.h"

#define MAX_SPEED 80

// fired every joystick message, uses the message to set motor powers on robot
inline void moveRobot( const sensor_msgs::Joy &joy_msg) {
  motorSet(6, joy_msg.buttons[7] * MAX_SPEED - joy_msg.buttons[5] * MAX_SPEED);
  motorSet(7, joy_msg.buttons[6] * MAX_SPEED - joy_msg.buttons[4] * MAX_SPEED);

  int lp = 0;
  int rp = 0;
  if(joy_msg.axes[0] < 0.0) { lp = -MAX_SPEED; rp = MAX_SPEED; }
  else if(joy_msg.axes[0] > 0.0) { lp = MAX_SPEED; rp = -MAX_SPEED; }
  else if(joy_msg.axes[1] < 0.0) { lp = -MAX_SPEED; rp = -MAX_SPEED; }
  else if(joy_msg.axes[1] > 0.0) { lp = MAX_SPEED; rp = MAX_SPEED; }
  else { 
    lp = MAX_SPEED * (joy_msg.axes[3] + joy_msg.axes[2]);
    rp = MAX_SPEED * (joy_msg.axes[3] - joy_msg.axes[2]);
  }
  
  lp = (abs(lp) > 10) ? lp : 0;
  rp = (abs(rp) > 10) ? rp : 0;

  motorSet(1, -lp);
  motorSet(10, rp);
}

inline void begin(void*){
  // set up nodehandle instance and a subscriber for Joystick events
  ros::NodeHandle nh;
  ros::Subscriber<sensor_msgs::Joy> sub("joy", &moveRobot);
    
  nh.initNode();
  nh.subscribe(sub);

  // loop for subscriber does not need delay - this ensures the node handle is as reliable as possible.
  while (1) {
    nh.spinOnce();
  }
}

// is a setup function.
inline void setup()
{
  taskCreate(&begin, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_HIGHEST);
}

