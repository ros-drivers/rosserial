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

#include "ros.h"
#include "WProgram.h"

#define MODE_FIRST_FF       0
#define MODE_SECOND_FF      1
#define MODE_TOPIC          2   // waiting for topic id
#define MODE_TYPE           3   // waiting for topic type
#define MODE_SIZE_H         4   // waiting for message size
#define MODE_SIZE_L         5   
#define MODE_MESSAGE        6

/* 
 * Publishers 
 */
ros::Publisher::Publisher(const char * topic_name, Msg * msg )
{
  id_ = 0;
  topic_ = topic_name;
  msg_ = msg;
}

int ros::Publisher::publish(Msg * msg)
{
  if(!nh_->configured_)
    return 0;
  return nh_->publish(id_, msg);
}


/* 
 * Subscribers
 */
ros::Subscriber::Subscriber(const char * topic_name, Msg * msg, msgCb *callback)
{
  id_ = 0;
  topic_ = topic_name;
  msg_ = msg;
  cb_ = callback; 
}


/* 
 * Node Handles
 */
bool ros::NodeHandle::advertise(Publisher & p)
{
  int i;
  for(i = 0; i < MAX_PUBLISHERS; i++)
  {
    if(publishers[i] == 0) // empty slot
    {
      publishers[i] = &p;
      p.id_ = i+1;
      p.nh_ = this;
      return true;
    }
  }
  return false;
}

bool ros::NodeHandle::subscribe(Subscriber & s)
{
  int i;
  for(i = 0; i < MAX_SUBSCRIBERS; i++)
  {
    if(subscribers[i] == 0) // empty slot
    {
      subscribers[i] = &s;
      s.id_ = i+128;
      s.nh_ = this;
      return true;
    }
  }
  return false;
}

void ros::NodeHandle::negotiateTopics()
{
  int i;
  for(i = 0; i < MAX_PUBLISHERS; i++)
  {
    if(publishers[i] != 0) // non-empty slot
    {
      // send publisher information
      makeHeader();
      WRITE( (unsigned char) 0 );
      WRITE( strlen(publishers[i]->topic_) + strlen(publishers[i]->msg_->getType()) + 3 );
      WRITE( publishers[i]->id_ );  
      WRITE( strlen(publishers[i]->topic_) );
      PRINT( publishers[i]->topic_ );
      WRITE( strlen(publishers[i]->msg_->getType()) );
      PRINT( publishers[i]->msg_->getType() );
    }
  }
  for(i = 0; i < MAX_SUBSCRIBERS; i++)
  {
    if(subscribers[i] != 0) // non-empty slot
    {
      // send subscriber information
      makeHeader();
      WRITE( (unsigned char) 0 );
      WRITE( strlen(subscribers[i]->topic_) + strlen(subscribers[i]->msg_->getType()) + 3 );
      WRITE( subscribers[i]->id_ );  
      WRITE( strlen(subscribers[i]->topic_) );
      PRINT( subscribers[i]->topic_ );
      WRITE( strlen(subscribers[i]->msg_->getType()) );
      PRINT( subscribers[i]->msg_->getType() );
    }
  }
  configured_ = true;
}

int ros::NodeHandle::publish(int id, Msg * msg)
{
  int l = msg->serialize(message_out) + 1;
  int chk = id + 0 + (l>>8) + (l&255);
  makeHeader();
  WRITE(id);
  WRITE( (unsigned char) 0 );
  WRITE( (unsigned char) l>>8);
  WRITE( (unsigned char) l&255);
  for(int i = 0; i<l-1; i++)
  {
    WRITE(message_out[i]);
    chk += message_out[i];
  }
  WRITE( 255 - (chk%256) );
  return 1;
}

inline void ros::NodeHandle::makeHeader()
{
  WRITE(0xff);
  WRITE(0xff);
}

void ros::NodeHandle::initNode()
{
  // initialize publisher and subscriber lists
  int i;;
  for(i = 0; i < MAX_PUBLISHERS; i++)
  {
    publishers[i] = 0;
    subscribers[i] = 0;
  }
  // initialize serial
  Serial.begin(57600);
  configured_ = false;
  mode_ = 0;
  bytes_ = 0;
  index_ = 0;
  topic_ = 0;
}

void ros::NodeHandle::spinOnce()
{
  /* while available buffer, read data */
  while( Serial.available() > 0 )
  {  
    int data = Serial.read();
    if( mode_ == MODE_MESSAGE ){        /* message data being recieved */
      message_in[index_++] = data;
      checksum_ += data;
      bytes_--;
      if(bytes_ == 0)                   /* is message complete? if so, pass along */
      {
        if(topic_ == 0)
        {
          negotiateTopics();
          FLUSH;
        }
        else
        {
          checksum_ = checksum_ % 256;
          if( checksum_ == 255){
              if(subscribers[topic_-128])
                subscribers[topic_-128]->cb_( message_in );
          }
        }
        mode_ = 0;
      }
    }else if( mode_ == MODE_FIRST_FF ){
      if(data == 0xff)
        mode_++;
    }else if( mode_ == MODE_SECOND_FF ){
      if(data == 0xff){
        mode_++;
      }else
        mode_ = MODE_FIRST_FF;
    }else if( mode_ == MODE_TOPIC ){    /* this is topic name */
      topic_ = data;
      if(topic_ == 0 || topic_ > 127)
        mode_++;
      else
        mode_ = MODE_FIRST_FF;
      checksum_ = data;
    }else if( mode_ == MODE_TYPE ){     /* this is the message type (topic or service) */
      mode_++;
      checksum_ += data;
    }else if( mode_ == MODE_SIZE_H ){   /* top half of message size */
      bytes_ = data<<8;
      mode_++;
      checksum_ += data;
    }else if( mode_ == MODE_SIZE_L ){   /* bottom half of message size */
      bytes_ += data;
      index_ = 0;
      mode_++;
      checksum_ += data;
    }
  }
}

