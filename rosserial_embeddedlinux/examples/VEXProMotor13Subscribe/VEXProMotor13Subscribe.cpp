/*
 * VEXProMotor13Subscribe.cpp
 *
 *  Created on: Jul 12, 2012
 *      Author: bouchier
 */

#include <ros.h>
#include <std_msgs/Int32.h>
#include <stdio.h>
#include "qemotoruser.h"

/*
 * Control motor 13 speed by publishing the desired speed on a ros topic with e.g.
 * $ rostopic pub my_topic std_msgs/Int32 120
 */

ros::NodeHandle  nh;
CQEMotorUser &motor = CQEMotorUser::GetRef();
char *rosSrvrIp = "192.168.15.149";

void messageCb(const std_msgs::Int32& motor13_msg){
	int speed = motor13_msg.data;
	printf("Received subscribed motor speed %d\n", speed);
    motor.SetPWM(0, speed);
}
ros::Subscriber<std_msgs::Int32> sub("motor13", messageCb );


int main()
{
	//nh.initNode();
	nh.initNode(rosSrvrIp);
	nh.subscribe(sub);

	while(1) {
		  sleep(1);
		  nh.spinOnce();
	}
}
