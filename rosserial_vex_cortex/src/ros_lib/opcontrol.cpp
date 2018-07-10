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
#include "helloworld.cpp"

/* 
 *
 *
 * NO GLOBAL OBJECT DEFINITIONS!
 *
 * DO NOT globally define either structs or objects.
 * See issue: https://github.com/purduesigbots/pros/issues/48
 * 
 * Instead, put all objects/structs inside functions.
 *
 *
 */

// entry point of user control period.
void operatorControl() {

	//running the hello world example
	setup();
	
	while(1) delay(20);
}
