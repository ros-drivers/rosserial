#include "main.h"

#include "ros.h"
#include "std_msgs/String.h"

void rosConnection(void* param) {
  ros::NodeHandle nh;
  std_msgs::String str_msg;
  char msgdata[20];

  ros::Publisher rob("rob", &str_msg);
  nh.initNode();
  nh.advertise(rob);

  str_msg.data = msgdata;

  while(1) {

    // publish a message including the time of publish
    sprintf(msgdata, "[%d] hello", (int) millis());
    rob.publish(&str_msg);
    nh.loginfo("Trying to publish on rob topic");
    nh.spinOnce();
    delay(500);
  }
}

void setup () {
  TaskHandle firstTaskHandle = taskCreate(rosConnection, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT);

  while(1) {
  pros::NodeHandle nh;
  std_msgs::String str_msg;
  char msgdata[20];

  ros::Publisher rob("rob", &str_msg);
  nh.initNode();
  nh.advertise(rob);

  str_msg.data = msgdata;

    // drive
    int power = joystickGetAnalog(1, 2);
    int power2 = joystickGetAnalog(1, 3);
    power = (abs(power) > 10) ? power : 0;
    power2 = (abs(power2) > 10) ? power2 : 0;
    motorSet(1, -power);
    motorSet(10, power2);

    // arm
    motorSet(7, joystickGetDigital(1, 6, JOY_UP) ? -127 : (joystickGetDigital(1, 6, JOY_DOWN) ? 127 : 0));
    motorSet(6, joystickGetDigital(1, 5, JOY_UP) ? -127 : (joystickGetDigital(1, 5, JOY_DOWN) ? 127 : 0));
    delay(20);
  }
}


