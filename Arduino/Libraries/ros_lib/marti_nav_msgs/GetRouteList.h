#ifndef _ROS_SERVICE_GetRouteList_h
#define _ROS_SERVICE_GetRouteList_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "marti_nav_msgs/Route.h"

namespace marti_nav_msgs
{

static const char GETROUTELIST[] = "marti_nav_msgs/GetRouteList";

  class GetRouteListRequest : public ros::Msg
  {
    public:

    GetRouteListRequest()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return GETROUTELIST; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetRouteListResponse : public ros::Msg
  {
    public:
      uint8_t routes_length;
      marti_nav_msgs::Route st_routes;
      marti_nav_msgs::Route * routes;
      bool success;
      const char* message;

    GetRouteListResponse():
      routes_length(0), routes(NULL),
      success(0),
      message("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = routes_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < routes_length; i++){
      offset += this->routes[i].serialize(outbuffer + offset);
      }
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
      uint8_t routes_lengthT = *(inbuffer + offset++);
      if(routes_lengthT > routes_length)
        this->routes = (marti_nav_msgs::Route*)realloc(this->routes, routes_lengthT * sizeof(marti_nav_msgs::Route));
      offset += 3;
      routes_length = routes_lengthT;
      for( uint8_t i = 0; i < routes_length; i++){
      offset += this->st_routes.deserialize(inbuffer + offset);
        memcpy( &(this->routes[i]), &(this->st_routes), sizeof(marti_nav_msgs::Route));
      }
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

    const char * getType(){ return GETROUTELIST; };
    const char * getMD5(){ return "24b443520442ddc540d0fd59f35403a5"; };

  };

  class GetRouteList {
    public:
    typedef GetRouteListRequest Request;
    typedef GetRouteListResponse Response;
  };

}
#endif
