/* 
 * rosserial::std_msgs::Float64 Test
 * Receives a Float64 input, subtracts 1.0, and publishes it
 */

#include <ros.h>
#include <std_msgs/Float64.h>

float x; 

ros::NodeHandle nh;

ROS_CALLBACK(messageCb, std_msgs::Float64, msg)
  x = msg.data - 1.0;
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
}

std_msgs::Float64 test;
ros::Subscriber s("your_topic", &msg, &messageCb);
ros::Publisher p("my_topic", &test);

void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.advertise(p);
  nh.subscribe(s);
}

void loop()
{
  test.data = x;
  p.publish( &test );
  nh.spinOnce();
  delay(10);
}

