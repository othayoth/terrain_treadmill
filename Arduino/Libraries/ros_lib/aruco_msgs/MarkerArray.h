#ifndef _ROS_aruco_msgs_MarkerArray_h
#define _ROS_aruco_msgs_MarkerArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "aruco_msgs/Marker.h"

namespace aruco_msgs
{

  class MarkerArray : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t markers_length;
      aruco_msgs::Marker st_markers;
      aruco_msgs::Marker * markers;

    MarkerArray():
      header(),
      markers_length(0), markers(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset++) = markers_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < markers_length; i++){
      offset += this->markers[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint8_t markers_lengthT = *(inbuffer + offset++);
      if(markers_lengthT > markers_length)
        this->markers = (aruco_msgs::Marker*)realloc(this->markers, markers_lengthT * sizeof(aruco_msgs::Marker));
      offset += 3;
      markers_length = markers_lengthT;
      for( uint8_t i = 0; i < markers_length; i++){
      offset += this->st_markers.deserialize(inbuffer + offset);
        memcpy( &(this->markers[i]), &(this->st_markers), sizeof(aruco_msgs::Marker));
      }
     return offset;
    }

    const char * getType(){ return "aruco_msgs/MarkerArray"; };
    const char * getMD5(){ return "9d486b76ee1f72a8b0d33e8c66a97306"; };

  };

}
#endif