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

#ifndef _ROSSERIAL_VEX_V5_V5_HARDWARE_H_
#define _ROSSERIAL_VEX_V5_V5_HARDWARE_H_

#include "main.h"
#include "ros_lib/rosserial_vex_v5/utils/RingBuf.h"

// for the mutex.
#include "pros/apix.h"

#define SERIAL_CLASS int
#define ROSVEX_BUFFER_INPUT_SIZE 32

using RB = RingBufCPP<char, ROSVEX_BUFFER_INPUT_SIZE>;

// load the serial reading into a buffer.
inline void vexRosBufferInput(void* arg) {

  void** arglist = (void**) arg;
  RB* inputBuffer = (RB*) arglist[0];
  __FILE* streamOut = (__FILE*) arglist[1];

  int readcount = 0;
  while(1) {
    char c =  fgetc(streamOut);
    inputBuffer->add(c);
  }
}

class V5Hardware {

  public:
    V5Hardware(): rosvexMutex(), inputBuffer(rosvexMutex), failCount(), successCount() {
    }

    // any initialization code necessary to use the serial port
    // note: the serial port initialization for rosserial for VEX Cortex must be implemented in `src/init.cpp` 
    // see that file for more information. 
    void init() {
      serctl(SERCTL_DISABLE_COBS, NULL);
      rosFile = fopen("/ser/sout", "r+");

      // not typesafe, be careful!
      void** taskArgs = (void**) malloc( sizeof(void*) * 2);
      taskArgs[0] = &inputBuffer;
      taskArgs[1] = rosFile;

      pros::Task reader(vexRosBufferInput, taskArgs);
    }

    // read a byte from the serial port. -1 = failure
    int read() {
      char c;
      // pull serial reading out of the buffer.
      if(inputBuffer.pull(&c)) {
        char sucmsg[16];
        return c;
      }

      return -1;
    }

    // write data to the connection to ROS
    void write(uint8_t* data, int length) {
      for(int i = 0; i < length; i++) {
        vexroswritechar(data[i]);
      }
    }
    // returns milliseconds since start of program
    unsigned long time() {
      return pros::c::millis();
    }
  private:
    int failCount;
    int successCount;
    pros::Mutex rosvexMutex;
    __FILE * rosFile;
    RB inputBuffer;

    // writing helper.
    void vexroswritechar(uint8_t data) {
      fputc(data, rosFile);
      fflush(rosFile);
    }
    
    // reading helper.
    char vexrosreadchar() {
      return fgetc(rosFile);
    }
    };

#endif
