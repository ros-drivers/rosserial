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

/* 
 * Author: Michael Ferguson
 */

#include <ros/time.h>
#include <time_fx.h>

namespace ros
{
  void normalizeSecNSec(unsigned long& sec, unsigned long& nsec){
    unsigned long nsec_part= nsec % 1000000000UL;
    unsigned long sec_part = nsec / 1000000000UL;
    sec += sec_part;
    nsec = nsec_part;
  }

  Time& Time::fromNSec(long t)
  {
    sec = t / 1000000000;
    nsec = t % 1000000000;
    normalizeSecNSec(sec, nsec);
    return *this;
  }

  Time& Time::operator +=(const Duration &rhs)
  {
    sec += rhs.sec;
    nsec += rhs.nsec;
    normalizeSecNSec(sec, nsec);
    return *this; 
  }

  Time& Time::operator -=(const Duration &rhs){
    sec += -rhs.sec;
    nsec += -rhs.nsec;
    normalizeSecNSec(sec, nsec);
    return *this;
  }

  static unsigned long sec_offset, nsec_offset;
  static Time current_time;

  Time Time::now(){
    unsigned long ms = millis();
    current_time.sec = ms/1000 + sec_offset;
    current_time.nsec = (ms%1000)*1000000UL + nsec_offset;
    normalizeSecNSec(current_time.sec, current_time.nsec);
    return current_time;
  }

  void Time::setNow( Time & new_now )
  {
    unsigned long ms = millis();
    sec_offset = new_now.sec - ms/1000 - 1;
    nsec_offset = new_now.nsec - (ms%1000)*1000000UL + 1000000000UL;
    normalizeSecNSec(sec_offset, nsec_offset);
  }

}
