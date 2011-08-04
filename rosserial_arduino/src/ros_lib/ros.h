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
 * ROS definitions for Arduino
 * Author: Michael Ferguson , Adam Stambler
 */

#ifndef ros_lib_h
#define ros_lib_h

#define MAX_PUBLISHERS      25
#define MAX_SUBSCRIBERS     25

#define BUFFER_SIZE         512

#define TOPIC_NEGOTIATION   0
#define TOPIC_PUBLISHERS    0
#define TOPIC_SUBSCRIBERS   1
#define TOPIC_SERVICES      3

#define TOPIC_TIME          10

#define SYNC_SECONDS        5


namespace ros
{

	/*
	 * Forward declaration of NodeHandleInterface
	 * This class takes care of all the node handle functions
	 * that an outside programmer would see.  NodeHandle
	 * takes the hardware template and performs all of the actual
	 * hardware templated functions.
	 */
	class NodeHandleInterface;

  /* Base Message Type */
  class Msg
  {
    public: 
      virtual int serialize(unsigned char *outbuffer) = 0;
	  virtual int deserialize(unsigned char *data) = 0;
      virtual const char * getType() = 0;
      
  };



  /* Generic Publisher */
  class Publisher
  {
    public:
      Publisher( const char * topic_name, Msg * msg );
      int publish( Msg * msg );

      int id_;     
      NodeHandleInterface * nh_;
      const char * topic_;
      Msg * msg_;
  };

  /* Base class for objects recieving messages (Services and Subscribers) */
class MsgReceiver{
	public:
		virtual void receive(unsigned char *data)=0;

		//Distinguishes between different receiver types
		virtual int _getType()=0;
		virtual const char * getMsgType()=0;
		int id_;
		NodeHandleInterface * nh_;
		const char * topic_;
};

/* ROS Subscriber
 * This class handles holding the msg so that
 * it is not continously reallocated.  It is also used by the
 * node handle to keep track of callback functions and IDs.
 * */
template<typename MsgT>
class Subscriber: public MsgReceiver{
public:

	typedef void(*CallbackT)(const MsgT&);

	Subscriber(const char * topic_name, CallbackT msgCB){
		topic_ = topic_name;
		cb_= msgCB;
	}
	MsgT msg;
	virtual void receive(unsigned char* data){
		msg.deserialize(data);
		this->cb_(msg);
	}
	virtual const char * getMsgType(){return this->msg.getType();}
	virtual int _getType(){return TOPIC_SUBSCRIBERS;}
private:
	CallbackT cb_;

};



}
#include <NodeHandleInterface.h>
#include "NodeHandle.h"

#endif
