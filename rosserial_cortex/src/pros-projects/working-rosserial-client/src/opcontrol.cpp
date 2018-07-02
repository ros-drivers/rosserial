/** @file opcontrol.c
 * @brief File for operator control code
 *
 * This file should contain the user operatorControl() function and any functions related to it.
 *
 * Any copyright is dedicated to the Public Domain.
 * http://creativecommons.org/publicdomain/zero/1.0/
 *
 * PROS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "main.h"

// EXAMPLES 

#include "./examples/helloworld.cpp"
// #include "./examples/subscriber.cpp"
// #include "./examples/logging.cpp"
// #include "./examples/timeandtf.cpp"
// #include "./examples/justdrive.cpp"
// #include "./examples/joydrive.cpp"

// running the sensor streamer
//#include "ros.h"
//#include "rosvexconfig.h"

// entry point of user control period.
void operatorControl() {
	delay(500);
	// ros::NodeHandle nh;
	// nh.initNode();
	// vexros::VexRosConfig conf(&nh);
	// conf.getConfigMessage();
	
	//running the examples
	setup();
	
	// ALWAYS END WITH WHILE LOOP
	while(1) delay(20);
}
