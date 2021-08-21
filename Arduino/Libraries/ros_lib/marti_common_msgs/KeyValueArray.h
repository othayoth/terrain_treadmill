#ifndef _ROS_marti_common_msgs_KeyValueArray_h
#define _ROS_marti_common_msgs_KeyValueArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "marti_common_msgs/KeyValue.h"

namespace marti_common_msgs
{

  class KeyValueArray : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t items_length;
      marti_common_msgs::KeyValue st_items;
      marti_common_msgs::KeyValue * items;

    KeyValueArray():
      header(),
      items_length(0), items(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset++) = items_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < items_length; i++){
      offset += this->items[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint8_t items_lengthT = *(inbuffer + offset++);
      if(items_lengthT > items_length)
        this->items = (marti_common_msgs::KeyValue*)realloc(this->items, items_lengthT * sizeof(marti_common_msgs::KeyValue));
      offset += 3;
      items_length = items_lengthT;
      for( uint8_t i = 0; i < items_length; i++){
      offset += this->st_items.deserialize(inbuffer + offset);
        memcpy( &(this->items[i]), &(this->st_items), sizeof(marti_common_msgs::KeyValue));
      }
     return offset;
    }

    const char * getType(){ return "marti_common_msgs/KeyValueArray"; };
    const char * getMD5(){ return "3b303032d5c2c08f75f9a40a839cb16c"; };

  };

}
#endif