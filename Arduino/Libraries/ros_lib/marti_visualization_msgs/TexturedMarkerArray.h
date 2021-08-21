#ifndef _ROS_marti_visualization_msgs_TexturedMarkerArray_h
#define _ROS_marti_visualization_msgs_TexturedMarkerArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "marti_visualization_msgs/TexturedMarker.h"

namespace marti_visualization_msgs
{

  class TexturedMarkerArray : public ros::Msg
  {
    public:
      uint8_t markers_length;
      marti_visualization_msgs::TexturedMarker st_markers;
      marti_visualization_msgs::TexturedMarker * markers;

    TexturedMarkerArray():
      markers_length(0), markers(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
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
      uint8_t markers_lengthT = *(inbuffer + offset++);
      if(markers_lengthT > markers_length)
        this->markers = (marti_visualization_msgs::TexturedMarker*)realloc(this->markers, markers_lengthT * sizeof(marti_visualization_msgs::TexturedMarker));
      offset += 3;
      markers_length = markers_lengthT;
      for( uint8_t i = 0; i < markers_length; i++){
      offset += this->st_markers.deserialize(inbuffer + offset);
        memcpy( &(this->markers[i]), &(this->st_markers), sizeof(marti_visualization_msgs::TexturedMarker));
      }
     return offset;
    }

    const char * getType(){ return "marti_visualization_msgs/TexturedMarkerArray"; };
    const char * getMD5(){ return "9a529a0eaa0a63d94d3445d26d3fe59a"; };

  };

}
#endif