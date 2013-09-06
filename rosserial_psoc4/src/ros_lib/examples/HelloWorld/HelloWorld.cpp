/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

#include "isnprintf.h"

ros::NodeHandle  nh;

void messageCb( const std_msgs::Empty& toggle_msg){
  nh.loginfo("callback");
}

ros::Subscriber<std_msgs::Empty> sub("toggle_led", messageCb );

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";


void setup()
{
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
}

void loop()
{
  str_msg.data = hello;
  isnprintf(hello+5, 7, "%d", (uint32_t)nh.now().sec);
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
}
