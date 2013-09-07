/* 
 * rosserial ADC Example
 * 
 * This is a poor man's Oscilloscope.  It does not have the sampling 
 * rate or accuracy of a commerical scope, but it is great to get
 * an analog value into ROS in a pinch.
 */

#include <ros.h>
#include <rosserial_psoc4/Adc.h>
#include "ADC_psoc4.h"

ros::NodeHandle nh;

rosserial_psoc4::Adc adc_msg;
ros::Publisher p("adc", &adc_msg);

uint32_t next_report_time;
const uint32_t kReportIntervalMs = 250;

void setup()
{ 
  nh.initNode();
  nh.advertise(p);
  next_report_time = SysTimer::millis();
  adc_setup();
}

//We average the analog reading to elminate some of the noise
int averageAnalog(int pin){
  int v=0;
  for(int i=0; i<4; i++) v += analogRead(pin);
  if(v<0) v = 0;
  return v/4;
}

void loop()
{
  if ((int32_t)(SysTimer::millis()-next_report_time) > 0) {
    next_report_time += kReportIntervalMs;

    adc_msg.adc0 = averageAnalog(0);
    adc_msg.adc1 = averageAnalog(1);
    adc_msg.adc2 = averageAnalog(2);
    adc_msg.adc3 = averageAnalog(3);
    adc_msg.adc4 = averageAnalog(4);
    adc_msg.adc5 = averageAnalog(5);
   
    p.publish(&adc_msg);
  }
  nh.spinOnce();

}

