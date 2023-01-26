/*
 * ros.h
 */

#ifndef _ROS_H_
#define _ROS_H_

#include "ros/node_handle.h"

#ifdef USE_USBCON
#include "RP2040_Hardware_USB.h"
#else
#include "RP2040_Hardware.h"
#endif

namespace ros
{
  #ifdef USE_USBCON
  typedef ros::NodeHandle_<RP2040_Hardware_USB> NodeHandle;
  #else
  typedef ros::NodeHandle_<RP2040_Hardware> NodeHandle;
  #endif
}

#endif
