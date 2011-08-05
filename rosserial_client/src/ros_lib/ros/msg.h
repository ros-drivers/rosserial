/*
 * Msg.h
 *
 *  Created on: Aug 5, 2011
 *      Author: astambler
 */

#ifndef ROS_MSG_H_
#define ROS_MSG_H_

namespace ros{
/* Base Message Type */
class Msg
{
  public:
    virtual int serialize(unsigned char *outbuffer) = 0;
	  virtual int deserialize(unsigned char *data) = 0;
    virtual const char * getType() = 0;

};
}

#endif /* MSG_H_ */
