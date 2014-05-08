// testdrive.cpp : An example of how to use rosserial in Windows
//

#include "stdafx.h"
#include <string>
#include <stdio.h>
#include "ros.h"
#include <geometry_msgs/Twist.h>
#include <windows.h>

using std::string;

int main(int argc, char* argv[])
{
	ros::NodeHandle  nh;
	if (argc != 2) {
		printf("Usage: testdrive host[:port]\n");
		return 0;
	}

	printf("Connecting to server at %s\n", argv[1]);
	nh.initNode(argv[1]);

	printf("Advertising cmd_vel message\n");
	geometry_msgs::Twist twist_msg;
	ros::Publisher cmd_vel_pub("husky/cmd_vel", &twist_msg);
	nh.advertise(cmd_vel_pub);

	printf("Go husky go!\n");
	while (1) {
		twist_msg.linear.x = 5.1f;
		twist_msg.angular.z = -1.8f;
		cmd_vel_pub.publish(&twist_msg);

		nh.spinOnce();
		Sleep(100);
	}
	return 0;
}
