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

#ifndef _ROSSERIAL_VEX_V5_V5_V5RS485_H_
#define _ROSSERIAL_VEX_V5_V5_V5RS485_H_

#include "pros/apix.h"

class V5RS485 {
   public:
    V5RS485(int readPortNum = 19, int writePortNum = 20, int baud = 115200)
        : readPort(readPortNum, baud),
          writePort(writePortNum, baud) {}

    void init() {
        pros::delay(10);
        readPort.flush();
        writePort.flush();
    }

    // read a byte from the serial port. -1 = failure
    int read() { 
        int read = readPort.read_byte();
        return read;
    }

    // write data to the connection to ROS
    void write(uint8_t* data, int length) { 
        int freeBytes = writePort.get_write_free();

        if(freeBytes > length) { // Enough bytes free in buffer
            writePort.write(data, length);
        } else {
            printf("Serial buffer full!\n");
            writePort.write(data, freeBytes);
            for(int i = freeBytes; i < length; i++) {
                while(writePort.get_write_free() == 0) {
                    pros::delay(5);
                }
                writePort.write_byte(data[i]);
            }
        }
    }

    // returns milliseconds since start of program
    unsigned long time() { return pros::c::millis(); }

   private:
    pros::Serial readPort;
    pros::Serial writePort;
};

#endif