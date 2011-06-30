/* 

 Blink the Arduino LED
 */

#include <ros.h>
#include <std_msgs/Empty.h>


ros::NodeHandle nh;
std_msgs::Empty toggle_msg;


ROS_CALLBACK(messageCb, std_msgs::Empty, msg)

  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
}

ros::Subscriber sub("toggle_led", &toggle_msg, messageCb );


void setup()
{ 
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);

}

void loop()
{  
  nh.spinOnce();
  delay(1);
}

