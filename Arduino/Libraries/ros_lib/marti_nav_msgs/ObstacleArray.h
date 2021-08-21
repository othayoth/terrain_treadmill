#ifndef _ROS_marti_nav_msgs_ObstacleArray_h
#define _ROS_marti_nav_msgs_ObstacleArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "marti_nav_msgs/Obstacle.h"

namespace marti_nav_msgs
{

  class ObstacleArray : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t obstacles_length;
      marti_nav_msgs::Obstacle st_obstacles;
      marti_nav_msgs::Obstacle * obstacles;

    ObstacleArray():
      header(),
      obstacles_length(0), obstacles(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset++) = obstacles_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < obstacles_length; i++){
      offset += this->obstacles[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint8_t obstacles_lengthT = *(inbuffer + offset++);
      if(obstacles_lengthT > obstacles_length)
        this->obstacles = (marti_nav_msgs::Obstacle*)realloc(this->obstacles, obstacles_lengthT * sizeof(marti_nav_msgs::Obstacle));
      offset += 3;
      obstacles_length = obstacles_lengthT;
      for( uint8_t i = 0; i < obstacles_length; i++){
      offset += this->st_obstacles.deserialize(inbuffer + offset);
        memcpy( &(this->obstacles[i]), &(this->st_obstacles), sizeof(marti_nav_msgs::Obstacle));
      }
     return offset;
    }

    const char * getType(){ return "marti_nav_msgs/ObstacleArray"; };
    const char * getMD5(){ return "2af80afb1cab2b611e3a96daa3ee0f01"; };

  };

}
#endif