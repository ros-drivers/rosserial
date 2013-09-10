/*
 * rosserial PubSub Example
 * Prints "hello world!" and toggles led
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

#include "BlueLed_psoc4.h"
#include "isnprintf.h"

ros::NodeHandle  nh;


bool ledOn;
void messageCb( const std_msgs::Empty& toggle_msg){
  ledOn = !ledOn;   // blink the led
  setBlueLed(ledOn);
}

ros::Subscriber<std_msgs::Empty> sub("toggle_led", messageCb );

uint32_t next_report_time;
const uint32_t kReportIntervalMs = 5000;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

int32_t loop_count;

void setup()
{
  blueLed_setup();
  ledOn = false;
  setBlueLed(ledOn);
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
  next_report_time = SysTimer::millis();
}

void loop()
{
  if ((int32_t)(SysTimer::millis()-next_report_time) > 0) {
    next_report_time += kReportIntervalMs;
    str_msg.data = hello;
    chatter.publish( &str_msg );
    isnprintf(hello+6, 5, "%d", loop_count++);
  }
  nh.spinOnce();
}
