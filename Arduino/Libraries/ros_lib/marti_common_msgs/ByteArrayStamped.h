#ifndef _ROS_marti_common_msgs_ByteArrayStamped_h
#define _ROS_marti_common_msgs_ByteArrayStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace marti_common_msgs
{

  class ByteArrayStamped : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t value_length;
      int8_t st_value;
      int8_t * value;

    ByteArrayStamped():
      header(),
      value_length(0), value(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset++) = value_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < value_length; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_valuei;
      u_valuei.real = this->value[i];
      *(outbuffer + offset + 0) = (u_valuei.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->value[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint8_t value_lengthT = *(inbuffer + offset++);
      if(value_lengthT > value_length)
        this->value = (int8_t*)realloc(this->value, value_lengthT * sizeof(int8_t));
      offset += 3;
      value_length = value_lengthT;
      for( uint8_t i = 0; i < value_length; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_st_value;
      u_st_value.base = 0;
      u_st_value.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->st_value = u_st_value.real;
      offset += sizeof(this->st_value);
        memcpy( &(this->value[i]), &(this->st_value), sizeof(int8_t));
      }
     return offset;
    }

    const char * getType(){ return "marti_common_msgs/ByteArrayStamped"; };
    const char * getMD5(){ return "375ed7aa29ecfbdffa16b36b36760a28"; };

  };

}
#endif