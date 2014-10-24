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

#ifndef _ROS_PUBLISHER_H_
#define _ROS_PUBLISHER_H_

#include "rosserial_msgs/TopicInfo.h"
#include "node_handle.h"
#include "msg.h"

namespace ros {
  
  /* Base class for objects publishers */
  class Publisher_
  {
    public:
      Publisher_( Msg * msg ) : msg_(msg) {};
      virtual int publish( const Msg * msg ) = 0;
      virtual int getEndpointType() = 0;
      
      virtual const char * getTopic() = 0;

      Msg *msg_;
      // id_ and no_ are set by NodeHandle when we advertise 
      int id_;
      NodeHandleBase_* nh_;
  };
  
  /* Generic Publisher */
  template<typename T_ConstStringType>
  class PublisherTempl : public Publisher_
  {
    public:
      PublisherTempl( T_ConstStringType topic_name, Msg * msg, int endpoint=rosserial_msgs::TopicInfo::ID_PUBLISHER) :
	Publisher_( msg ),
        topic_(topic_name), 
        endpoint_(endpoint) {};

      virtual int publish( const Msg * msg ) { return nh_->publish(id_, msg); };
      virtual int getEndpointType(){ return endpoint_; }
      
      virtual const char * getTopic() { return ros::StringConverter::convertToConstChar( topic_ ); };

    private:
      T_ConstStringType topic_;
      int endpoint_;
  };
  
  /*
   * for backwards compatibility
   */
  typedef PublisherTempl<const char *> Publisher;
}

#endif
