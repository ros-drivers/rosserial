#include "ros.h"

#include "sensor_msgs/Joy.h"

#define MAX_SPEED 80

// run continuously by the setup function, publishes the time.
void loop(ros::NodeHandle & nha)
{
  nha.spinOnce();
}

// fired every joystick message
void moveRobot( const sensor_msgs::Joy &joy_msg) {
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

// is a setup function.
void setup()
{
  ros::NodeHandle nh;

  ros::Subscriber<sensor_msgs::Joy> sub("joy", &moveRobot);
    
  nh.initNode();
  nh.subscribe(sub);

  while (1) {
    loop(nh);
  }
}
