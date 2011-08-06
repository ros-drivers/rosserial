/*
 * msg_receiver.h
 *
 *  Created on: Aug 5, 2011
 *      Author: astambler
 */

#ifndef MSG_RECEIVER_H_
#define MSG_RECEIVER_H_

namespace ros{

/* Base class for objects recieving messages (Services and Subscribers) */
  class MsgReceiver
  {
	public:
		virtual void receive(unsigned char *data)=0;

		//Distinguishes between different receiver types
		virtual int _getType()=0;
		virtual const char * getMsgType()=0;
		int id_;
		const char * topic_;
  };
}

#endif /* MSG_RECEIVER_H_ */
