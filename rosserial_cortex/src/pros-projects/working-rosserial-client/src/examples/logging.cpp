#include <ros.h>
#include "main.h"

void loop(ros::NodeHandle & nha)
{
  //wait until you are actually connected
  while (!nha.connected())
  {
    loggydebug("not connected...");
    nha.spinOnce();
  }
 
  //Now you can send all of the statements you want
  //to be logged at the appropriate verbosity level
  nha.logdebug("Debug Statement");
  nha.loginfo("Program info");
  nha.logwarn("Warnings.");
  nha.logerror("Errors..");
  nha.logfatal("Fatalities!");
  delay(5000);
}

void setup()
{
  ros::NodeHandle nh;
  nh.initNode();
  while(1){
    loop(nh);
  }
}
