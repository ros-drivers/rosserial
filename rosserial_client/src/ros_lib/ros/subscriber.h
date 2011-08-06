/*
 * subscriber.h
 *
 *  Created on: Aug 5, 2011
 *      Author: astambler
 */

#ifndef SUBSCRIBER_H_
#define SUBSCRIBER_H_

#include "rosserial_ids.h"
#include "msg_receiver.h"
namespace ros{

/* ROS Subscriber
 * This class handles holding the msg so that
 * it is not continously reallocated.  It is also used by the
 * node handle to keep track of callback functions and IDs.
 */
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
#endif /* SUBSCRIBER_H_ */
