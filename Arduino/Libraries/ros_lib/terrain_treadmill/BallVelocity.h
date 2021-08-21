#ifndef _ROS_terrain_treadmill_BallVelocity_h
#define _ROS_terrain_treadmill_BallVelocity_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace terrain_treadmill
{

  class BallVelocity : public ros::Msg
  {
    public:
      std_msgs::Header header;
      float xdot;
      float ydot;
      float wdot;
      float dt;

    BallVelocity():
      header(),
      xdot(0),
      ydot(0),
      wdot(0),
      dt(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_xdot;
      u_xdot.real = this->xdot;
      *(outbuffer + offset + 0) = (u_xdot.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_xdot.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_xdot.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_xdot.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->xdot);
      union {
        float real;
        uint32_t base;
      } u_ydot;
      u_ydot.real = this->ydot;
      *(outbuffer + offset + 0) = (u_ydot.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ydot.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ydot.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ydot.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ydot);
      union {
        float real;
        uint32_t base;
      } u_wdot;
      u_wdot.real = this->wdot;
      *(outbuffer + offset + 0) = (u_wdot.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wdot.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_wdot.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_wdot.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wdot);
      union {
        float real;
        uint32_t base;
      } u_dt;
      u_dt.real = this->dt;
      *(outbuffer + offset + 0) = (u_dt.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dt.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dt.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dt.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dt);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_xdot;
      u_xdot.base = 0;
      u_xdot.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_xdot.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_xdot.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_xdot.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->xdot = u_xdot.real;
      offset += sizeof(this->xdot);
      union {
        float real;
        uint32_t base;
      } u_ydot;
      u_ydot.base = 0;
      u_ydot.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ydot.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ydot.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ydot.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ydot = u_ydot.real;
      offset += sizeof(this->ydot);
      union {
        float real;
        uint32_t base;
      } u_wdot;
      u_wdot.base = 0;
      u_wdot.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_wdot.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_wdot.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_wdot.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->wdot = u_wdot.real;
      offset += sizeof(this->wdot);
      union {
        float real;
        uint32_t base;
      } u_dt;
      u_dt.base = 0;
      u_dt.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dt.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dt.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dt.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->dt = u_dt.real;
      offset += sizeof(this->dt);
     return offset;
    }

    const char * getType(){ return "terrain_treadmill/BallVelocity"; };
    const char * getMD5(){ return "a613c6fc405ff07ef738eae595cd2f87"; };

  };

}
#endif