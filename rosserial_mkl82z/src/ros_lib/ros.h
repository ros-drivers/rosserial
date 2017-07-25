#ifndef _ROS_H_
#define _ROS_H_

#include "ros/node_handle.h"
#include "mkl82z_hardware.h"

namespace ros
{
  typedef NodeHandle_<Mkl82zHardware> NodeHandle;
}

#endif
