/*
 * rosserial Publisher Example for VEX Cortex
 * Prints "hello world!"
 */

#include <ros.h>
#include <std_msgs/String.h>

// run continuously by the setup function, publishes the time.
inline void loop(ros::NodeHandle & nh, ros::Publisher & p, std_msgs::String & str_msg, char* msgdata)
{
  str_msg.data = msgdata;
  p.publish( &str_msg );
  nh.spinOnce();
  delay(250);
}

// is a setup function.
inline void setup()
{
  loggy("\n\n\n\n\r\t\tROSserial for VEX Cortex V2 - June 2018 - START\n\n\n\n\n\r");

  ros::NodeHandle  nh;
  std_msgs::String str_msg;
  ros::Publisher chatter("chatter\0", &str_msg);

  nh.initNode();
  nh.advertise(chatter);

  char* msg = (char*) malloc(20 * sizeof(char));
  while (1) {

    // send a message about the time!
    sprintf(msg, "[%d] Hello there!!", (int) millis());
    loop(nh, chatter, str_msg, msg);
  }
}
