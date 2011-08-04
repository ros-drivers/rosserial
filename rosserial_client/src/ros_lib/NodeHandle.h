/*
 * NodeHandle.h
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

#ifndef NODEHANDLE_H_
#define NODEHANDLE_H_

#include "std_msgs/Time.h"

#define MODE_FIRST_FF       0
#define MODE_SECOND_FF      1
#define MODE_TOPIC_L        2   // waiting for topic id
#define MODE_TOPIC_H        3
#define MODE_SIZE_L         4   // waiting for message size
#define MODE_SIZE_H         5
#define MODE_MESSAGE        6
#define MODE_CHECKSUM       7

namespace ros {

  /* Node Handle */
  template<class Hardware>
  class NodeHandle_ : public NodeHandleInterface
  {

    protected:
      Hardware hardware_;

      /* time used for syncing */
      unsigned long rt_time;

      /* used for computing current time */
      unsigned long sec_offset, nsec_offset;

    public:
	  NodeHandle_(Hardware hardware){
        hardware_ = hardware;
	  }
	  NodeHandle_(){}

	  void setHardware(Hardware& h){
        hardware_ = h;
      }
	  Hardware& getHardware(){
		return hardware_;
	  }

      /* Start serial, initialize buffers */
      virtual void initNode(){
        hardware_.init();
        configured_ = false;
        mode_ = 0;
        bytes_ = 0;
        index_ = 0;
        topic_ = 0;
        total_recievers=0;
      };

      /* This function goes in your loop() function, it handles
       *  serial input and callbacks for subscribers.
       */
      virtual void spinOnce(){
        /* restart if timed-out */
        if((hardware_.time() - last_msg_receive_time) > 500){
          mode_ == MODE_FIRST_FF;
          if((hardware_.time() - last_sync_receive_time) > (SYNC_SECONDS*2200) ){
            configured_ = false;
          }
        }

        /* while available buffer, read data */
        while( true )
        {
          int data = hardware_.read();
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
              last_msg_receive_time = hardware_.time();
            }
          }else if( mode_ == MODE_SECOND_FF ){
            if(data == 0xff){
              mode_++;
            }else{
              mode_ = MODE_FIRST_FF;
            }
          }else if( mode_ == MODE_TOPIC_L ){  /* bottom half of topic id */
            topic_ = data;
            mode_++;
            checksum_ = data;                 /* first byte included in checksum */
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
              }else if(topic_ == TOPIC_TIME){
                syncTime(message_in);
              }else{
                if(receivers[topic_-100])
                  receivers[topic_-100]->receive( message_in );
              }
            }
            mode_ = MODE_FIRST_FF;
          }
        }

        /* occasionally sync time */
        if( configured_ && ((hardware_.time()-last_sync_time) > (SYNC_SECONDS*900) )){
          requestSyncTime();
          last_sync_time = hardware_.time();
        }
      }

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
        message_out[6+l] = 255 - (chk%256);

        hardware_.write(message_out, 6+l+1);
        return 1;
      }

      /**************************************************************
       * Time functions
       */

      void requestSyncTime()
      {
        std_msgs::Time t;
        publish( TOPIC_TIME, &t);
        rt_time = hardware_.time();
      }

      void syncTime( unsigned char * data )
      {
        std_msgs::Time t;
        unsigned long offset = hardware_.time() - rt_time;

        t.deserialize(data);

        t.data.sec += offset/1000;
        t.data.nsec += (offset%1000)*1000000UL;

        this->setNow(t.data);
        last_sync_receive_time = hardware_.time();
      }

      virtual unsigned long hardwareTime(){
        return this->hardware_.time();
      }

      virtual Time now(){
        unsigned long ms = hardware_.time();
        Time current_time;
        current_time.sec = ms/1000 + sec_offset;
        current_time.nsec = (ms%1000)*1000000UL + nsec_offset;
        normalizeSecNSec(current_time.sec, current_time.nsec);
        return current_time;
      }

      virtual void setNow( Time & new_now )
      {
        unsigned long ms = hardware_.time();
        sec_offset = new_now.sec - ms/1000 - 1;
        nsec_offset = new_now.nsec - (ms%1000)*1000000UL + 1000000000UL;
        normalizeSecNSec(sec_offset, nsec_offset);
      }

  };

}

#endif /* NODEHANDLE_H_ */
