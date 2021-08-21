#ifndef _ROS_SERVICE_UpdateRouteMetadata_h
#define _ROS_SERVICE_UpdateRouteMetadata_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "marti_nav_msgs/RoutePoint.h"

namespace marti_nav_msgs
{

static const char UPDATEROUTEMETADATA[] = "marti_nav_msgs/UpdateRouteMetadata";

  class UpdateRouteMetadataRequest : public ros::Msg
  {
    public:
      const char* route_guid;
      uint8_t metadata_points_length;
      marti_nav_msgs::RoutePoint st_metadata_points;
      marti_nav_msgs::RoutePoint * metadata_points;

    UpdateRouteMetadataRequest():
      route_guid(""),
      metadata_points_length(0), metadata_points(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_route_guid = strlen(this->route_guid);
      memcpy(outbuffer + offset, &length_route_guid, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->route_guid, length_route_guid);
      offset += length_route_guid;
      *(outbuffer + offset++) = metadata_points_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < metadata_points_length; i++){
      offset += this->metadata_points[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_route_guid;
      memcpy(&length_route_guid, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_route_guid; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_route_guid-1]=0;
      this->route_guid = (char *)(inbuffer + offset-1);
      offset += length_route_guid;
      uint8_t metadata_points_lengthT = *(inbuffer + offset++);
      if(metadata_points_lengthT > metadata_points_length)
        this->metadata_points = (marti_nav_msgs::RoutePoint*)realloc(this->metadata_points, metadata_points_lengthT * sizeof(marti_nav_msgs::RoutePoint));
      offset += 3;
      metadata_points_length = metadata_points_lengthT;
      for( uint8_t i = 0; i < metadata_points_length; i++){
      offset += this->st_metadata_points.deserialize(inbuffer + offset);
        memcpy( &(this->metadata_points[i]), &(this->st_metadata_points), sizeof(marti_nav_msgs::RoutePoint));
      }
     return offset;
    }

    const char * getType(){ return UPDATEROUTEMETADATA; };
    const char * getMD5(){ return "4326dd3985865ba6412643260ac9da6f"; };

  };

  class UpdateRouteMetadataResponse : public ros::Msg
  {
    public:
      bool success;
      const char* message;

    UpdateRouteMetadataResponse():
      success(0),
      message("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      uint32_t length_message = strlen(this->message);
      memcpy(outbuffer + offset, &length_message, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->message, length_message);
      offset += length_message;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
      uint32_t length_message;
      memcpy(&length_message, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_message; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_message-1]=0;
      this->message = (char *)(inbuffer + offset-1);
      offset += length_message;
     return offset;
    }

    const char * getType(){ return UPDATEROUTEMETADATA; };
    const char * getMD5(){ return "937c9679a518e3a18d831e57125ea522"; };

  };

  class UpdateRouteMetadata {
    public:
    typedef UpdateRouteMetadataRequest Request;
    typedef UpdateRouteMetadataResponse Response;
  };

}
#endif
