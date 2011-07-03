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
#include "ros/time.h"

#include "std_msgs/Time.h"
#include "rosserial_msgs/TopicInfo.h"

#define MODE_FIRST_FF       0
#define MODE_SECOND_FF      1
#define MODE_TOPIC_L        2   // waiting for topic id
#define MODE_TOPIC_H        3
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
      p.id_ = i+100+MAX_SUBSCRIBERS;
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
      s.id_ = i+100;
      s.nh_ = this;
      return true;
    }
  }
  return false;
}

void ros::NodeHandle::negotiateTopics()
{
  rosserial_msgs::TopicInfo ti; 
  int i;
  for(i = 0; i < MAX_PUBLISHERS; i++)
  {
    if(publishers[i] != 0) // non-empty slot
    {
      ti.topic_id = publishers[i]->id_;
      ti.topic_name = (unsigned char *) publishers[i]->topic_;
      ti.message_type = (unsigned char *) publishers[i]->msg_->getType();
      publish( TOPIC_PUBLISHERS, &ti );
    }
  }
  for(i = 0; i < MAX_SUBSCRIBERS; i++)
  {
    if(subscribers[i] != 0) // non-empty slot
    {
      ti.topic_id = subscribers[i]->id_;
      ti.topic_name = (unsigned char *) subscribers[i]->topic_;
      ti.message_type = (unsigned char *) subscribers[i]->msg_->getType();
      publish( TOPIC_SUBSCRIBERS, &ti );
    }
  }
  configured_ = true;
}

static unsigned long rt_time;
void ros::NodeHandle::requestSyncTime()
{
  std_msgs::Time t;
  publish( TOPIC_TIME, &t);
  rt_time = millis();
}

static unsigned long last_sync_time;
static unsigned long last_sync_receive_time;
static unsigned long last_msg_receive_time;
void ros::NodeHandle::syncTime( unsigned char * data )
{
  std_msgs::Time t;
  unsigned long offset = millis() - rt_time;

  t.deserialize(data);

  t.data.sec += offset/1000;
  t.data.nsec += (offset%1000)*1000000UL;

  ros::Time::setNow(t.data);
  last_sync_receive_time = millis();
}

int ros::NodeHandle::publish(int id, Msg * msg)
{
  int l = msg->serialize(message_out);
  int chk = (id&255) + (id>>8) + (l&255) + (l>>8);
    for(int i = 0; i<l; i++)
  {
    chk += message_out[i];
  }
   chk = 255 - (chk%256);
   
  fx_putc(0xff);
  fx_putc(0xff);
  fx_putc( (unsigned char) id&255);
  fx_putc( (unsigned char) id>>8);
  fx_putc( (unsigned char) l&255);
  fx_putc( (unsigned char) l>>8);
  for(int i = 0; i<l; i++)
  {
    fx_putc(message_out[i]);
  }
  fx_putc(  chk);
  return 1;
}


void ros::NodeHandle::initNode()
{
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
  /* restart if timed-out */
  if((millis() - last_msg_receive_time) > 500){
    mode_ == MODE_FIRST_FF;
    if((millis() - last_sync_receive_time) > (SYNC_SECONDS*2200) ){
      //configured_ = false;
    }
  }

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
      if(data == 0xff){
        mode_++;
        last_msg_receive_time = millis();
      }
    }else if( mode_ == MODE_SECOND_FF ){
      if(data == 0xff){
        mode_++;
      }else
        mode_ = MODE_FIRST_FF;
    }else if( mode_ == MODE_TOPIC_L ){  /* bottom half of topic id */
      topic_ = data; 
      mode_++;
      checksum_ = data;                 // first byte included in checksum
    }else if( mode_ == MODE_TOPIC_H ){  /* top half of topic id */
      topic_ += data<<8;
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
        if(topic_ == TOPIC_NEGOTIATION){
		  requestSyncTime();
          negotiateTopics();
          last_sync_time = 0;
        }
        else if(topic_ == TOPIC_TIME)
        {
          syncTime(message_in);
        }
        else
        {
          if(subscribers[topic_-100])
            subscribers[topic_-100]->cb_( message_in );
        }
      }
      mode_ = MODE_FIRST_FF;
    }
  }
   
  /* occasionally sync time */
  if( configured_ && ((millis()-last_sync_time) > (SYNC_SECONDS*900) )){
    requestSyncTime();
	last_sync_time = millis();
  }
}

