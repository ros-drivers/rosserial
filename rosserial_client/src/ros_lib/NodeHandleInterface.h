/*
 * NodeHandleInterface.h
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
 *
 * Author: Michael Ferguson , Adam Stambler
 */

#ifndef NODEHANDLEINTERFACE_H_
#define NODEHANDLEINTERFACE_H_

#include "ros/time.h"

/*
 * This is the base node handle class used by
 * Publishers and subscribers to access the node handle.
 * The node handle is not used directly because it is templated.
 */

namespace ros {

  class NodeHandleInterface{
    protected:
      void makeHeader();
      Publisher * publishers[MAX_PUBLISHERS];
      MsgReceiver * receivers[MAX_SUBSCRIBERS];

      int mode_;
      int bytes_;
      int topic_;
      int index_;
      int checksum_;
	  bool configured_;

      int total_recievers;

      unsigned char message_in[BUFFER_SIZE];
      unsigned char message_out[BUFFER_SIZE];

      /* used for syncing the time */
      unsigned long last_sync_time;
      unsigned long last_sync_receive_time;
      unsigned long last_msg_receive_time;

    public:
      /* Initialize the node and hardware */
      virtual void initNode()=0;

      /*  Let the node handle its IO.  This is where the callbacks happen. */
      virtual void spinOnce()=0;

      /* Publish a message */
      virtual int publish(int id, Msg * msg)=0;

      /* Advertise a publisher */
      bool advertise(Publisher &p);

      /* Register a subscriber with the node */
      template<typename MsgT>
      bool subscribe(Subscriber< MsgT> &s){
        if (total_recievers >= MAX_SUBSCRIBERS) return false;
        receivers[total_recievers] = (MsgReceiver*)&s;
        s.id_ = 100+total_recievers;
        total_recievers++;
        return true;
      }

      /* Establish the topics to be sent across the serial connection*/
      void negotiateTopics();
    
      /* Are we connected to the PC? */
      bool connected();

	  virtual Time now() =0;
      virtual void setNow(Time& t)=0;
  };

}

#endif /* NODEHANDLEINTERFACE_H_ */
