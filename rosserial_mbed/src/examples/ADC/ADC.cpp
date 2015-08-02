/*
 * rosserial ADC Example
 *
 * This is a poor man's Oscilloscope.  It does not have the sampling
 * rate or accuracy of a commerical scope, but it is great to get
 * an analog value into ROS in a pinch.
 */

#include "mbed.h"
#include <ros.h>
#include <rosserial_mbed/Adc.h>

ros::NodeHandle nh;

rosserial_mbed::Adc adc_msg;
ros::Publisher p("adc", &adc_msg);



//We average the analog reading to elminate some of the noise
int averageAnalog(PinName pin) {
    int v=0;
    for (int i=0; i<4; i++) v+= AnalogIn(pin).read_u16();
    return v/4;
}

long adc_timer;

int main() {
    nh.initNode();

    nh.advertise(p);

    while (1) {
        adc_msg.adc0 = averageAnalog(p15);
        adc_msg.adc1 = averageAnalog(p16);
        adc_msg.adc2 = averageAnalog(p17);
        adc_msg.adc3 = averageAnalog(p18);
        adc_msg.adc4 = averageAnalog(p19);
        adc_msg.adc5 = averageAnalog(p20);

        p.publish(&adc_msg);

        nh.spinOnce();
    }
}

