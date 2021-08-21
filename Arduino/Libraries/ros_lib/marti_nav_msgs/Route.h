#ifndef _ROS_marti_nav_msgs_Route_h
#define _ROS_marti_nav_msgs_Route_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "marti_nav_msgs/RoutePoint.h"
#include "marti_common_msgs/KeyValue.h"

namespace marti_nav_msgs
{

  class Route : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t route_points_length;
      marti_nav_msgs::RoutePoint st_route_points;
      marti_nav_msgs::RoutePoint * route_points;
      uint8_t properties_length;
      marti_common_msgs::KeyValue st_properties;
      marti_common_msgs::KeyValue * properties;

    Route():
      header(),
      route_points_length(0), route_points(NULL),
      properties_length(0), properties(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset++) = route_points_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < route_points_length; i++){
      offset += this->route_points[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset++) = properties_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < properties_length; i++){
      offset += this->properties[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint8_t route_points_lengthT = *(inbuffer + offset++);
      if(route_points_lengthT > route_points_length)
        this->route_points = (marti_nav_msgs::RoutePoint*)realloc(this->route_points, route_points_lengthT * sizeof(marti_nav_msgs::RoutePoint));
      offset += 3;
      route_points_length = route_points_lengthT;
      for( uint8_t i = 0; i < route_points_length; i++){
      offset += this->st_route_points.deserialize(inbuffer + offset);
        memcpy( &(this->route_points[i]), &(this->st_route_points), sizeof(marti_nav_msgs::RoutePoint));
      }
      uint8_t properties_lengthT = *(inbuffer + offset++);
      if(properties_lengthT > properties_length)
        this->properties = (marti_common_msgs::KeyValue*)realloc(this->properties, properties_lengthT * sizeof(marti_common_msgs::KeyValue));
      offset += 3;
      properties_length = properties_lengthT;
      for( uint8_t i = 0; i < properties_length; i++){
      offset += this->st_properties.deserialize(inbuffer + offset);
        memcpy( &(this->properties[i]), &(this->st_properties), sizeof(marti_common_msgs::KeyValue));
      }
     return offset;
    }

    const char * getType(){ return "marti_nav_msgs/Route"; };
    const char * getMD5(){ return "626dfe06202116afac99e6de9fa42b3e"; };

  };

}
#endif