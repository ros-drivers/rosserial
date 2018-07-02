#ifndef _ROS_actionlib_TwoIntsGoal_h
#define _ROS_actionlib_TwoIntsGoal_h

#include "vexstrlen.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace actionlib
{

  class TwoIntsGoal : public ros::Msg
  {
    public:
      typedef int64_t _a_type;
      _a_type a;
      typedef int64_t _b_type;
      _b_type b;

    TwoIntsGoal():
      a(0),
      b(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int64_t real;
        uint64_t base;
      } u_a;
      u_a.real = this->a;
      *(outbuffer + offset + 0) = (u_a.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_a.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_a.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_a.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_a.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_a.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_a.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_a.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->a);
      union {
        int64_t real;
        uint64_t base;
      } u_b;
      u_b.real = this->b;
      *(outbuffer + offset + 0) = (u_b.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_b.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_b.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_b.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_b.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_b.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_b.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_b.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->b);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int64_t real;
        uint64_t base;
      } u_a;
      u_a.base = 0;
      u_a.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_a.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_a.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_a.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_a.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_a.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_a.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_a.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->a = u_a.real;
      offset += sizeof(this->a);
      union {
        int64_t real;
        uint64_t base;
      } u_b;
      u_b.base = 0;
      u_b.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_b.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_b.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_b.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_b.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_b.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_b.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_b.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->b = u_b.real;
      offset += sizeof(this->b);
     return offset;
    }

    const char * getType(){ return "actionlib/TwoIntsGoal"; };
    const char * getMD5(){ return "36d09b846be0b371c5f190354dd3153e"; };

  };

}
#endif