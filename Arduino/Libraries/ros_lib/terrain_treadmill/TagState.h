#ifndef _ROS_terrain_treadmill_TagState_h
#define _ROS_terrain_treadmill_TagState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace terrain_treadmill
{

  class TagState : public ros::Msg
  {
    public:
      std_msgs::Header header;
      float x;
      float y;
      float theta;
      float xdot;
      float ydot;
      float thetadot;
      float dt;

    TagState():
      header(),
      x(0),
      y(0),
      theta(0),
      xdot(0),
      ydot(0),
      thetadot(0),
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
      } u_x;
      u_x.real = this->x;
      *(outbuffer + offset + 0) = (u_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x);
      union {
        float real;
        uint32_t base;
      } u_y;
      u_y.real = this->y;
      *(outbuffer + offset + 0) = (u_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y);
      union {
        float real;
        uint32_t base;
      } u_theta;
      u_theta.real = this->theta;
      *(outbuffer + offset + 0) = (u_theta.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_theta.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_theta.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_theta.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->theta);
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
      } u_thetadot;
      u_thetadot.real = this->thetadot;
      *(outbuffer + offset + 0) = (u_thetadot.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_thetadot.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_thetadot.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_thetadot.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->thetadot);
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
      } u_x;
      u_x.base = 0;
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->x = u_x.real;
      offset += sizeof(this->x);
      union {
        float real;
        uint32_t base;
      } u_y;
      u_y.base = 0;
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->y = u_y.real;
      offset += sizeof(this->y);
      union {
        float real;
        uint32_t base;
      } u_theta;
      u_theta.base = 0;
      u_theta.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_theta.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_theta.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_theta.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->theta = u_theta.real;
      offset += sizeof(this->theta);
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
      } u_thetadot;
      u_thetadot.base = 0;
      u_thetadot.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_thetadot.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_thetadot.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_thetadot.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->thetadot = u_thetadot.real;
      offset += sizeof(this->thetadot);
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

    const char * getType(){ return "terrain_treadmill/TagState"; };
    const char * getMD5(){ return "8ae589b23e0a520f614756b5e0d06612"; };

  };

}
#endif