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
#include "ros/time.h"

#include "std_msgs/Time.h"
#include "rosserial_msgs/TopicInfo.h"

ros::NodeHandleInterface * __nh =0;

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
 * Node Handles
 */
bool ros::NodeHandleInterface::advertise(Publisher & p)
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

void ros::NodeHandleInterface::negotiateTopics()
{
  rosserial_msgs::TopicInfo ti; 
  int i;
  for(i = 0; i < MAX_PUBLISHERS; i++)
  {
    if(publishers[i] != 0) // non-empty slot
    {
      ti.topic_id = publishers[i]->id_;
      ti.topic_name = (char *) publishers[i]->topic_;
      ti.message_type = (char *) publishers[i]->msg_->getType();
      publish( TOPIC_PUBLISHERS, &ti );
    }
  }
  for(i = 0; i < MAX_SUBSCRIBERS; i++)
  {
    if(receivers[i] != 0) // non-empty slot
    {
      ti.topic_id = receivers[i]->id_;
      ti.topic_name = (char *) receivers[i]->topic_;
      ti.message_type = (char *) receivers[i]->getMsgType();
      publish( TOPIC_SUBSCRIBERS, &ti );
    }
  }
  configured_ = true;
}


