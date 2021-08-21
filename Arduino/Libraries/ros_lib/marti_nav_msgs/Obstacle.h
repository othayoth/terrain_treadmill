#ifndef _ROS_marti_nav_msgs_Obstacle_h
#define _ROS_marti_nav_msgs_Obstacle_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"

namespace marti_nav_msgs
{

  class Obstacle : public ros::Msg
  {
    public:
      const char* id;
      geometry_msgs::Pose pose;
      uint8_t polygon_length;
      geometry_msgs::Point st_polygon;
      geometry_msgs::Point * polygon;

    Obstacle():
      id(""),
      pose(),
      polygon_length(0), polygon(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_id = strlen(this->id);
      memcpy(outbuffer + offset, &length_id, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->id, length_id);
      offset += length_id;
      offset += this->pose.serialize(outbuffer + offset);
      *(outbuffer + offset++) = polygon_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < polygon_length; i++){
      offset += this->polygon[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_id;
      memcpy(&length_id, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_id-1]=0;
      this->id = (char *)(inbuffer + offset-1);
      offset += length_id;
      offset += this->pose.deserialize(inbuffer + offset);
      uint8_t polygon_lengthT = *(inbuffer + offset++);
      if(polygon_lengthT > polygon_length)
        this->polygon = (geometry_msgs::Point*)realloc(this->polygon, polygon_lengthT * sizeof(geometry_msgs::Point));
      offset += 3;
      polygon_length = polygon_lengthT;
      for( uint8_t i = 0; i < polygon_length; i++){
      offset += this->st_polygon.deserialize(inbuffer + offset);
        memcpy( &(this->polygon[i]), &(this->st_polygon), sizeof(geometry_msgs::Point));
      }
     return offset;
    }

    const char * getType(){ return "marti_nav_msgs/Obstacle"; };
    const char * getMD5(){ return "6379634b2f186de37a480e1f3f9b2e7f"; };

  };

}
#endif