/* 
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2022, Paul Tervoort
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
 
#include "pico/stdlib.h"

#include <stdio.h>

#ifndef ROS_RP2040_Hardware_USB_H_
#define ROS_RP2040_Hardware_USB_H_

class RP2040_Hardware_USB
{
public:
  RP2040_Hardware_USB() {}

  // any initialization code necessary to use the serial port
  void init() 
  {
    stdio_usb_init();
  }

  // read a byte from the serial port. -1 = failure
  int read() 
  {
    return getchar_timeout_us(0);
  }

  // write data to the connection to ROS
  void write(uint8_t* data, int length)
  {
    for(int i = 0; i < length; i++)
    {
      putchar_raw(data[i]);
    }
    //printf("%.*s", length, data);
  }

  // returns milliseconds since start of program
  unsigned long time()
  {
    return us_to_ms(time_us_32());
  }
};

#endif
