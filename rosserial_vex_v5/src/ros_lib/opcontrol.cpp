/** @file opcontrol.cpp
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
#include "ros_lib/rosserial_vex_v5/examples/helloworld.h"

// entry point of user control period.
void opcontrol() {

	//running the hello world example
	setup();
	
	while(1) pros::c::delay(20);
}
