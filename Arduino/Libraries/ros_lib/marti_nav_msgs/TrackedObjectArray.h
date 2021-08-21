#ifndef _ROS_marti_nav_msgs_TrackedObjectArray_h
#define _ROS_marti_nav_msgs_TrackedObjectArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "marti_nav_msgs/TrackedObject.h"

namespace marti_nav_msgs
{

  class TrackedObjectArray : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t objects_length;
      marti_nav_msgs::TrackedObject st_objects;
      marti_nav_msgs::TrackedObject * objects;

    TrackedObjectArray():
      header(),
      objects_length(0), objects(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset++) = objects_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < objects_length; i++){
      offset += this->objects[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint8_t objects_lengthT = *(inbuffer + offset++);
      if(objects_lengthT > objects_length)
        this->objects = (marti_nav_msgs::TrackedObject*)realloc(this->objects, objects_lengthT * sizeof(marti_nav_msgs::TrackedObject));
      offset += 3;
      objects_length = objects_lengthT;
      for( uint8_t i = 0; i < objects_length; i++){
      offset += this->st_objects.deserialize(inbuffer + offset);
        memcpy( &(this->objects[i]), &(this->st_objects), sizeof(marti_nav_msgs::TrackedObject));
      }
     return offset;
    }

    const char * getType(){ return "marti_nav_msgs/TrackedObjectArray"; };
    const char * getMD5(){ return "eeceef8756dddeb96eb4d6e607b3e5fd"; };

  };

}
#endif