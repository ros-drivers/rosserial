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

#ifndef ROS_NODEOUTPUT_H_
#define ROS_NODEOUTPUT_H_

#include "msg.h"

namespace ros {

  /*
   * This class is responsible for controlling the node ouput.
   * It it is the object that is passed to Publishers and services
   */
  class NodeOutput_{
    public:
      virtual int publish(int id, Msg* msg)=0;
  };

  template<class Hardware, int OUTSIZE =512>
  class NodeOutput : public NodeOutput_{

    private:
      Hardware* hardware_;
      bool configured_;
      unsigned char message_out[OUTSIZE];

    public:
      NodeOutput(Hardware* h){
        hardware_ = h;
        configured_ = false;
      }

      NodeOutput(){};

      void setHardware(Hardware* h){
        hardware_  = h;
        configured_=false;
      }
       
      void setConfigured(bool b){
        configured_ =b;
      }
      bool configured(){return configured_;};

      virtual int publish(int id, Msg * msg){
        if(!configured_) return 0;

        /* serialize message */
        int l = msg->serialize(message_out+6);

        /* setup the header */
        message_out[0] = 0xff;
        message_out[1] = 0xff;
        message_out[2] = (unsigned char) id&255;
        message_out[3] = (unsigned char) id>>8;
        message_out[4] = (unsigned char) l&255;
        message_out[5] = ((unsigned char) l>>8);

        /* calculate checksum */
        int chk = 0;
        for(int i =2; i<l+6; i++)
          chk += message_out[i];
        l += 6;
        message_out[l++] = 255 - (chk%256);

        hardware_->write(message_out, l);
        return l;
      }
  };

}

#endif
