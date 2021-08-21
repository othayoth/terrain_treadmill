#ifndef _ROS_marti_nav_msgs_GridMap_h
#define _ROS_marti_nav_msgs_GridMap_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/Image.h"

namespace marti_nav_msgs
{

  class GridMap : public ros::Msg
  {
    public:
      std_msgs::Header header;
      geometry_msgs::Point top_left;
      geometry_msgs::Point top_right;
      geometry_msgs::Point bottom_right;
      geometry_msgs::Point bottom_left;
      uint8_t map_names_length;
      char* st_map_names;
      char* * map_names;
      uint8_t map_data_length;
      sensor_msgs::Image st_map_data;
      sensor_msgs::Image * map_data;

    GridMap():
      header(),
      top_left(),
      top_right(),
      bottom_right(),
      bottom_left(),
      map_names_length(0), map_names(NULL),
      map_data_length(0), map_data(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->top_left.serialize(outbuffer + offset);
      offset += this->top_right.serialize(outbuffer + offset);
      offset += this->bottom_right.serialize(outbuffer + offset);
      offset += this->bottom_left.serialize(outbuffer + offset);
      *(outbuffer + offset++) = map_names_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < map_names_length; i++){
      uint32_t length_map_namesi = strlen(this->map_names[i]);
      memcpy(outbuffer + offset, &length_map_namesi, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->map_names[i], length_map_namesi);
      offset += length_map_namesi;
      }
      *(outbuffer + offset++) = map_data_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < map_data_length; i++){
      offset += this->map_data[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->top_left.deserialize(inbuffer + offset);
      offset += this->top_right.deserialize(inbuffer + offset);
      offset += this->bottom_right.deserialize(inbuffer + offset);
      offset += this->bottom_left.deserialize(inbuffer + offset);
      uint8_t map_names_lengthT = *(inbuffer + offset++);
      if(map_names_lengthT > map_names_length)
        this->map_names = (char**)realloc(this->map_names, map_names_lengthT * sizeof(char*));
      offset += 3;
      map_names_length = map_names_lengthT;
      for( uint8_t i = 0; i < map_names_length; i++){
      uint32_t length_st_map_names;
      memcpy(&length_st_map_names, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_map_names; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_map_names-1]=0;
      this->st_map_names = (char *)(inbuffer + offset-1);
      offset += length_st_map_names;
        memcpy( &(this->map_names[i]), &(this->st_map_names), sizeof(char*));
      }
      uint8_t map_data_lengthT = *(inbuffer + offset++);
      if(map_data_lengthT > map_data_length)
        this->map_data = (sensor_msgs::Image*)realloc(this->map_data, map_data_lengthT * sizeof(sensor_msgs::Image));
      offset += 3;
      map_data_length = map_data_lengthT;
      for( uint8_t i = 0; i < map_data_length; i++){
      offset += this->st_map_data.deserialize(inbuffer + offset);
        memcpy( &(this->map_data[i]), &(this->st_map_data), sizeof(sensor_msgs::Image));
      }
     return offset;
    }

    const char * getType(){ return "marti_nav_msgs/GridMap"; };
    const char * getMD5(){ return "3b88254125f8a40bfc5628e3d7439242"; };

  };

}
#endif