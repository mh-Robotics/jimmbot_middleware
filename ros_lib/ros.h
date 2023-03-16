#ifndef _ROS_H_
#define _ROS_H_

#include "ESP32Hardware.h"
#include "ros/node_handle.h"

namespace ros {
typedef NodeHandle_<ESP32Hardware, 25, 25, 1024, 1024>
    NodeHandle; // default 25, 25, 512, 512
}

#endif
