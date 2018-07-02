/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/Empty.h>


// will this mess up the stack?
void messageCb( const std_msgs::Empty& toggle_msg){
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  loggy("BLINK");
}


void loop(ros::NodeHandle & nha)
{
  nha.spinOnce();
  delay(1);
}

void setup()
{
  ros::NodeHandle nh;
  ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  while(1) {
    loop(nh);
  }
}
