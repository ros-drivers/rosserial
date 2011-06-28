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
 * Author: Michael Ferguson
 */

#ifndef ros_lib_h
#define ros_lib_h

#define MAX_PUBLISHERS      25
#define MAX_SUBSCRIBERS     25

#define BUFFER_SIZE         512

#define TOPIC_NEGOTIATION   0
#define TOPIC_PUBLISHERS    0
#define TOPIC_SUBSCRIBERS   1
// services?
#define TOPIC_TIME          10

#define SYNC_SECONDS        5

namespace ros
{
  class NodeHandle;

  /* Base Message Type */
  class Msg
  {
    public: 
      virtual int serialize(unsigned char *outbuffer) = 0;
	  virtual int deserialize(unsigned char *data) = 0;
      virtual const char * getType() = 0;
      
  };

  typedef void (msgCb) (unsigned char * data);

  /* Generic Publisher */
  class Publisher
  {
    public:
      Publisher( const char * topic_name, Msg * msg );
      int publish( Msg * msg );

      int id_;     
      NodeHandle * nh_;
      const char * topic_;
      Msg * msg_;
  };

  /* Generic Subscriber */
  class Subscriber
  {
    public:
      Subscriber( const char * topic_name, Msg * msg, msgCb * callback);

      int id_;     
      NodeHandle * nh_;
      const char * topic_;
      Msg * msg_;
      msgCb * cb_;
  };

  /* Node Handle */
  class NodeHandle
  {
    public:
      bool advertise(Publisher &p);
      bool subscribe(Subscriber &s);
      void negotiateTopics();
      int publish(int id, Msg * msg);

      /* synchronize time with host */
      void requestSyncTime();
      void syncTime(unsigned char * data);

      /* Start serial, initialize buffers */
      void initNode();

      /* This function goes in your loop() function, it handles 
       *  serial input and callbacks for subscribers. 
       */
      void spinOnce();

      bool configured_;

    private:
      void makeHeader();
      Publisher * publishers[MAX_PUBLISHERS];
      Subscriber * subscribers[MAX_SUBSCRIBERS];

      int mode_;
      int bytes_;
      int topic_;
      int index_;
      int checksum_;
	
      unsigned char message_in[BUFFER_SIZE];
      unsigned char message_out[BUFFER_SIZE];
  };

}

#define ROS_CALLBACK( fn, ty, message )\
ty message; \
void fn (unsigned char *data){\
    message.deserialize(data); 

#endif
