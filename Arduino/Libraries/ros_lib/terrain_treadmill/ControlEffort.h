#ifndef _ROS_terrain_treadmill_ControlEffort_h
#define _ROS_terrain_treadmill_ControlEffort_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace terrain_treadmill
{

  class ControlEffort : public ros::Msg
  {
    public:
      std_msgs::Header header;
      float Ux;
      float Uy;
      float Ut;

    ControlEffort():
      header(),
      Ux(0),
      Uy(0),
      Ut(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->Ux);
      offset += serializeAvrFloat64(outbuffer + offset, this->Uy);
      offset += serializeAvrFloat64(outbuffer + offset, this->Ut);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->Ux));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->Uy));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->Ut));
     return offset;
    }

    const char * getType(){ return "terrain_treadmill/ControlEffort"; };
    const char * getMD5(){ return "b8e063ad1d0cc0f542a2849f1f2e8b1b"; };

  };

}
#endif