#ifndef _ROS_nav_msgs_GetMapResult_h
#define _ROS_nav_msgs_GetMapResult_h

#include "vexstrlen.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "nav_msgs/OccupancyGrid.h"

namespace nav_msgs
{

  class GetMapResult : public ros::Msg
  {
    public:
      typedef nav_msgs::OccupancyGrid _map_type;
      _map_type map;

    GetMapResult():
      map()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->map.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->map.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "nav_msgs/GetMapResult"; };
    const char * getMD5(){ return "6cdd0a18e0aff5b0a3ca2326a89b54ff"; };

  };

}
#endif