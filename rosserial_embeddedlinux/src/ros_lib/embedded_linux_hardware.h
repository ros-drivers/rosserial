/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
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

#ifndef ROS_EMBEDDED_LINUX_HARDWARE_H_
#define ROS_EMBEDDED_LINUX_HARDWARE_H_

#include <iostream>

#ifdef BUILD_LIBROSSERIALEMBEDDEDLINUX
extern "C" int elCommInit(char *portName, int baud);
extern "C" int elCommRead(int fd);
extern "C" elCommWrite(int fd, uint8_t* data, int length);
#endif

#define DEFAULT_PORT "/dev/ttyAM1"

class EmbeddedLinuxHardware
{
public:
  EmbeddedLinuxHardware(const char *pn, long baud = 57600)
  {
    strncpy(portName, pn, 30);
    baud_ = baud;
  }

  EmbeddedLinuxHardware()
  {
    const char *envPortName = getenv("ROSSERIAL_PORT");
    if (envPortName == NULL)
      strcpy(portName, DEFAULT_PORT);
    else
      strncpy(portName, envPortName, 29);
    portName[29] = '\0'; // in case user gave us too long a port name
    baud_ = 57600;
  }

  void setBaud(long baud)
  {
    this->baud_ = baud;
  }

  int getBaud()
  {
    return baud_;
  }

  void init()
  {
    fd = elCommInit(portName, baud_);
    if (fd < 0)
    {
      std::cout << "Exiting" << std::endl;
      exit(-1);
    }
    std::cout << "EmbeddedHardware.h: opened serial port successfully\n";
    clock_gettime(CLOCK_MONOTONIC, &start);     // record when the program started
  }

  void init(const char *pName)
  {
    fd = elCommInit(pName, baud_);
    if (fd < 0)
    {
      std::cout << "Exiting" << std::endl;
      exit(-1);
    }
    std::cout << "EmbeddedHardware.h: opened comm port successfully\n";
    clock_gettime(CLOCK_MONOTONIC, &start);     // record when the program started
  }

  int read()
  {
    int c = elCommRead(fd);
    return c;
  }

  void write(uint8_t* data, int length)
  {
    elCommWrite(fd, data, length);
  }

  unsigned long time()
  {
    long millis, seconds, nseconds;

    clock_gettime(CLOCK_MONOTONIC, &end);

    seconds  = end.tv_sec  - start.tv_sec;
    nseconds = end.tv_nsec - start.tv_nsec;

    millis = ((seconds) * 1000 + nseconds / 1000000.0) + 0.5;

    return millis;
  }

protected:
  int fd;
  char portName[30];
  long baud_;
  struct timespec start, end;
};

#endif
