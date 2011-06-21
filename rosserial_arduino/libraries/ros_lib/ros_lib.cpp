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
#include "serial_fx.h"

#define MODE_FIRST_FF       0
#define MODE_SECOND_FF      1
#define MODE_TYPE           2   // waiting for topic type
#define MODE_TOPIC          3   // waiting for topic id
#define MODE_SIZE_L         4   // waiting for message size
#define MODE_SIZE_H         5   
#define MODE_MESSAGE        6
#define MODE_CHECKSUM       7

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
      int checksum = PKT_NEGOTIATION;
      // send publisher information
      makeHeader();                             // header
      fx_putc((unsigned char)PKT_NEGOTIATION);  // type
      // topic id
      int val = publishers[i]->id_;             // id
      fx_putc( (unsigned char) val );
      checksum += val; 
      // send packet length
      val = strlen(publishers[i]->topic_) + strlen(publishers[i]->msg_->getType()) + 2;
      fx_putc( (unsigned char) val&255 );       // packet length
      checksum += val&255; 
      fx_putc( (unsigned char) val>>8 );
      checksum += val>>8; 
      // send topic name
      val = strlen(publishers[i]->topic_);
      fx_putc( (unsigned char) val );           // name length
      checksum += val;
      const char * c = publishers[i]->topic_;   // name
      while( *c ){
        fx_putc( *c );
        checksum += *c++;
      }
      // send topic type
      val = strlen(publishers[i]->msg_->getType());
      fx_putc( (unsigned char) val );           // type length  
      checksum += val;
      c = publishers[i]->msg_->getType();       // type
      while( *c ){
        fx_putc( *c );
        checksum += *c++;
      }
      // checksum
      fx_putc( 255 - (checksum%256) );
    }
  }
  for(i = 0; i < MAX_SUBSCRIBERS; i++)
  {
    if(subscribers[i] != 0) // non-empty slot
    {
      int checksum = PKT_NEGOTIATION;
      // send subscriber information
      makeHeader();                              // header
      fx_putc((unsigned char)PKT_NEGOTIATION);   // type
      // topic id
      int val = subscribers[i]->id_;             // id
      fx_putc( (unsigned char) val );
      checksum += val; 
      // send packet length
      val = strlen(subscribers[i]->topic_) + strlen(subscribers[i]->msg_->getType()) + 2;
      fx_putc( (unsigned char) val&255 );       // packet length
      checksum += val&255;
      fx_putc( (unsigned char) val>>8 );
      checksum += val>>8; 
      // send topic name
      val = strlen(subscribers[i]->topic_);
      fx_putc( (unsigned char) val );            // name length
      checksum += val;
      const char * c = subscribers[i]->topic_;   // name
      while( *c ){
        fx_putc( *c );
        checksum += *c++;
      }
      // send topic type
      val = strlen(subscribers[i]->msg_->getType());
      fx_putc( (unsigned char) val );            // type length 
      checksum += val;  
      c = subscribers[i]->msg_->getType();       // type
      while( *c ){
        fx_putc( *c );
        checksum += *c++;
      }
      // checksum
      fx_putc( 255 - (checksum%256) );
    }
  }
  configured_ = true;
}

int ros::NodeHandle::publish(int id, Msg * msg)
{
  int l = msg->serialize(message_out) + 1;
  int chk = PKT_TOPIC + id + (l&255) + (l>>8);
  makeHeader();
  fx_putc( (unsigned char) PKT_TOPIC ); 
  fx_putc(id);
  fx_putc( (unsigned char) l&255);
  fx_putc( (unsigned char) l>>8);
  for(int i = 0; i<l-1; i++)
  {
    fx_putc(message_out[i]);
    chk += message_out[i];
  }
  fx_putc( 255 - (chk%256) );
  return 1;
}

inline void ros::NodeHandle::makeHeader()
{
  fx_putc(0xff);
  fx_putc(0xff);
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
  fx_open();
  configured_ = false;
  mode_ = 0;
  bytes_ = 0;
  index_ = 0;
  topic_ = 0;
}

void ros::NodeHandle::spinOnce()
{
  /* while available buffer, read data */
  while( true )
  {  
    int data = fx_getc();
    if( data < 0 )
      break;
    checksum_ += data;
    if( mode_ == MODE_MESSAGE ){        /* message data being recieved */
      message_in[index_++] = data;
      bytes_--;
      if(bytes_ == 0)                   /* is message complete? if so, checksum */
        mode_ = MODE_CHECKSUM;
    }else if( mode_ == MODE_FIRST_FF ){
      if(data == 0xff)
        mode_++;
    }else if( mode_ == MODE_SECOND_FF ){
      if(data == 0xff){
        mode_++;
      }else
        mode_ = MODE_FIRST_FF;
    }else if( mode_ == MODE_TYPE ){     /* this is the message type (topic or service) */
      type_ = data;
      mode_++;
      checksum_ = data;
    }else if( mode_ == MODE_TOPIC ){    /* this is topic name */
      topic_ = data;
      mode_++;
    }else if( mode_ == MODE_SIZE_L ){   /* bottom half of message size */
      bytes_ = data;
      index_ = 0;
      mode_++;
    }else if( mode_ == MODE_SIZE_H ){   /* top half of message size */
      bytes_ += data<<8;
      mode_ = MODE_MESSAGE;
      if(bytes_ == 0) 
        mode_ = MODE_CHECKSUM;
    }else if( mode_ == MODE_CHECKSUM ){ /* do checksum */
      if( (checksum_%256) == 255){
        if(type_ == PKT_NEGOTIATION){
          negotiateTopics();
        }
        else
        {
          if(subscribers[topic_-128])
            subscribers[topic_-128]->cb_( message_in );
        }
      }
      mode_ = MODE_FIRST_FF;
    }
  }
}

