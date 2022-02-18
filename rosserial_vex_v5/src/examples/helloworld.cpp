/*
 * rosserial Publisher Example for VEX Cortex
 * Prints "hello world!"
 */

#include "ros_lib/ros.h"
#include "ros_lib/std_msgs/String.h"

// this loop is run in setup function, which publishes  at 50hz.
void loop(ros::NodeHandle & nh, ros::Publisher & p, std_msgs::String & str_msg, char* msgdata)
{
  str_msg.data = msgdata;
  p.publish( &str_msg );
  nh.spinOnce();
  pros::c::delay(20);
}

// The setup function will start a publisher on the topic "chatter" and begin publishing there.
void setup()
{
  // debug logging
  // make a node handle object, string message, and publisher for that message.
  ros::NodeHandle  nh;
  std_msgs::String str_msg;
  ros::Publisher chatter("chatter\0", &str_msg);

  // set up rosserial, and prepare to publish the chatter message 
  nh.initNode();
  nh.advertise(chatter);

  // message data variable.
  char* msg = (char*) malloc(20 * sizeof(char));
  while (1) {

    // send a message about the time!
    sprintf(msg, "[%d] Hello there!!", (int) pros::c::millis());
    loop(nh, chatter, str_msg, msg);
  }
}
