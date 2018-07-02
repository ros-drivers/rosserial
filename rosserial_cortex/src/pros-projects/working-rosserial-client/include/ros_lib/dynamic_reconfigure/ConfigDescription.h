#ifndef _ROS_dynamic_reconfigure_ConfigDescription_h
#define _ROS_dynamic_reconfigure_ConfigDescription_h

#include "vexstrlen.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "dynamic_reconfigure/Group.h"
#include "dynamic_reconfigure/Config.h"

namespace dynamic_reconfigure
{

  class ConfigDescription : public ros::Msg
  {
    public:
      uint32_t groups_length;
      typedef dynamic_reconfigure::Group _groups_type;
      _groups_type st_groups;
      _groups_type * groups;
      typedef dynamic_reconfigure::Config _max_type;
      _max_type max;
      typedef dynamic_reconfigure::Config _min_type;
      _min_type min;
      typedef dynamic_reconfigure::Config _dflt_type;
      _dflt_type dflt;

    ConfigDescription():
      groups_length(0), groups(NULL),
      max(),
      min(),
      dflt()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->groups_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->groups_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->groups_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->groups_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->groups_length);
      for( uint32_t i = 0; i < groups_length; i++){
      offset += this->groups[i].serialize(outbuffer + offset);
      }
      offset += this->max.serialize(outbuffer + offset);
      offset += this->min.serialize(outbuffer + offset);
      offset += this->dflt.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t groups_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      groups_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      groups_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      groups_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->groups_length);
      if(groups_lengthT > groups_length)
        this->groups = (dynamic_reconfigure::Group*)realloc(this->groups, groups_lengthT * sizeof(dynamic_reconfigure::Group));
      groups_length = groups_lengthT;
      for( uint32_t i = 0; i < groups_length; i++){
      offset += this->st_groups.deserialize(inbuffer + offset);
        memcpy( &(this->groups[i]), &(this->st_groups), sizeof(dynamic_reconfigure::Group));
      }
      offset += this->max.deserialize(inbuffer + offset);
      offset += this->min.deserialize(inbuffer + offset);
      offset += this->dflt.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "dynamic_reconfigure/ConfigDescription"; };
    const char * getMD5(){ return "757ce9d44ba8ddd801bb30bc456f946f"; };

  };

}
#endif