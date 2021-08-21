#ifndef _ROS_marti_nav_msgs_RouteSpeedArray_h
#define _ROS_marti_nav_msgs_RouteSpeedArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "marti_nav_msgs/RouteSpeed.h"

namespace marti_nav_msgs
{

  class RouteSpeedArray : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t speeds_length;
      marti_nav_msgs::RouteSpeed st_speeds;
      marti_nav_msgs::RouteSpeed * speeds;

    RouteSpeedArray():
      header(),
      speeds_length(0), speeds(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset++) = speeds_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < speeds_length; i++){
      offset += this->speeds[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint8_t speeds_lengthT = *(inbuffer + offset++);
      if(speeds_lengthT > speeds_length)
        this->speeds = (marti_nav_msgs::RouteSpeed*)realloc(this->speeds, speeds_lengthT * sizeof(marti_nav_msgs::RouteSpeed));
      offset += 3;
      speeds_length = speeds_lengthT;
      for( uint8_t i = 0; i < speeds_length; i++){
      offset += this->st_speeds.deserialize(inbuffer + offset);
        memcpy( &(this->speeds[i]), &(this->st_speeds), sizeof(marti_nav_msgs::RouteSpeed));
      }
     return offset;
    }

    const char * getType(){ return "marti_nav_msgs/RouteSpeedArray"; };
    const char * getMD5(){ return "c5b2e8db78eaab7eafdb3ecf8d4e017f"; };

  };

}
#endif