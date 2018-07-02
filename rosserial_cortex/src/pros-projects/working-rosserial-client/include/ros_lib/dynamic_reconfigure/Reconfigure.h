#ifndef _ROS_SERVICE_Reconfigure_h
#define _ROS_SERVICE_Reconfigure_h
#include "vexstrlen.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "dynamic_reconfigure/Config.h"

namespace dynamic_reconfigure
{

static const char RECONFIGURE[] = "dynamic_reconfigure/Reconfigure";

  class ReconfigureRequest : public ros::Msg
  {
    public:
      typedef dynamic_reconfigure::Config _config_type;
      _config_type config;

    ReconfigureRequest():
      config()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->config.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->config.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return RECONFIGURE; };
    const char * getMD5(){ return "ac41a77620a4a0348b7001641796a8a1"; };

  };

  class ReconfigureResponse : public ros::Msg
  {
    public:
      typedef dynamic_reconfigure::Config _config_type;
      _config_type config;

    ReconfigureResponse():
      config()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->config.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->config.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return RECONFIGURE; };
    const char * getMD5(){ return "ac41a77620a4a0348b7001641796a8a1"; };

  };

  class Reconfigure {
    public:
    typedef ReconfigureRequest Request;
    typedef ReconfigureResponse Response;
  };

}
#endif
