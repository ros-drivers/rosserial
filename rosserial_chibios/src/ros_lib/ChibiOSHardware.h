/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote prducts derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROS_CHIBIOS_HARDWARE_H_
#define ROS_CHIBIOS_HARDWARE_H_

#include <hal.h>

class ChibiOSHardware {
public:
  void setPort(BaseChannel* io)
  {
    io_ = io;
  }

  BaseChannel* getPort()
  {
    return io_;
  }

  void init()
  {
  }

  int read()
  {
    return chnGetTimeout(io_, TIME_IMMEDIATE);
  }

  void write(uint8_t* data, int length)
  {
    chnWrite(io_, data, length);
  }

  unsigned long time()
  {
#if defined(OSAL_I2MS)
    return OSAL_I2MS(osalOsGetSystemTimeX());
#elif defined(TIME_I2MS)
    return TIME_I2MS(osalOsGetSystemTimeX());
#else
    return (osalOsGetSystemTimeX() * static_cast<systime_t>(1000)) / OSAL_ST_FREQUENCY;
#endif
  }

protected:
  BaseChannel* io_ = nullptr;
};

#endif
