#ifndef _ROS_terrain_treadmill_MotorVelocity_h
#define _ROS_terrain_treadmill_MotorVelocity_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace terrain_treadmill
{

  class MotorVelocity : public ros::Msg
  {
    public:
      float w1;
      float w2;
      float w3;

    MotorVelocity():
      w1(0),
      w2(0),
      w3(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_w1;
      u_w1.real = this->w1;
      *(outbuffer + offset + 0) = (u_w1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_w1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_w1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_w1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->w1);
      union {
        float real;
        uint32_t base;
      } u_w2;
      u_w2.real = this->w2;
      *(outbuffer + offset + 0) = (u_w2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_w2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_w2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_w2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->w2);
      union {
        float real;
        uint32_t base;
      } u_w3;
      u_w3.real = this->w3;
      *(outbuffer + offset + 0) = (u_w3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_w3.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_w3.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_w3.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->w3);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_w1;
      u_w1.base = 0;
      u_w1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_w1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_w1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_w1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->w1 = u_w1.real;
      offset += sizeof(this->w1);
      union {
        float real;
        uint32_t base;
      } u_w2;
      u_w2.base = 0;
      u_w2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_w2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_w2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_w2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->w2 = u_w2.real;
      offset += sizeof(this->w2);
      union {
        float real;
        uint32_t base;
      } u_w3;
      u_w3.base = 0;
      u_w3.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_w3.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_w3.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_w3.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->w3 = u_w3.real;
      offset += sizeof(this->w3);
     return offset;
    }

    const char * getType(){ return "terrain_treadmill/MotorVelocity"; };
    const char * getMD5(){ return "bd82aec785cf5731a7d25a0042e76809"; };

  };

}
#endif