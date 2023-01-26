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
#include "hardware/uart.h"

#ifndef ROS_RP2040_Hardware_H_
#define ROS_RP2040_Hardware_H_

class RP2040_Hardware
{
public:
  RP2040_Hardware(uart_inst_t* uart, int tx, int rx, long baud)
  {
    uart_ = uart;
    tx_ = tx;
    rx_ = rx;
    baud_ = baud;
  }
  RP2040_Hardware(long baud) : RP2040_Hardware(uart0, 0, 1, baud) {}
  RP2040_Hardware() : RP2040_Hardware(uart0, 0, 1, 57600) {}
  
  void custom_config(uart_inst_t* uart, int tx, int rx, long baud)
  {
    uart_ = uart;
    tx_ = tx;
    rx_ = rx;
    baud_ = baud;
  }

  // any initialization code necessary to use the serial port
  void init() 
  {
    uart_init(uart_, baud_);

    gpio_set_function(tx_, GPIO_FUNC_UART);
    gpio_set_function(rx_, GPIO_FUNC_UART);
  }

  // read a byte from the serial port. -1 = failure
  int read() 
  {
    if (uart_is_readable(uart_))
    {
      return uart_getc(uart_);
    }

    return -1;
  }

  // write data to the connection to ROS
  void write(uint8_t* data, int length)
  {
    uart_write_blocking(uart_, data, length);
  }

  // returns milliseconds since start of program
  unsigned long time()
  {
    return us_to_ms(time_us_32());
  }

protected:
  uart_inst_t* uart_;
  int rx_;
  int tx_;
  long baud_;
};

#endif
