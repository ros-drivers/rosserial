/*
 * ros.h
 */

#ifndef _ROS_H_
#define _ROS_H_

#include "ros/node_handle.h"
#include "RP2040_Hardware.h"
#include "RP2040_Hardware_USB.h"

namespace ros
{
  #ifdef USE_USBCON
  typedef ros::NodeHandle_<RP2040_Hardware_USB> NodeHandle;
  #else
  typedef ros::NodeHandle_<RP2040_Hardware> NodeHandle;
  #endif
}

#endif