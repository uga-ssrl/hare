#ifndef COMMON_INCLUDES_H
#define COMMON_INCLUDES_H

#include <climits>
#include <cstdint>
#include <stdlib.h>
#include <time.h>
#include <ros/ros.h>
#include <vector>
#include <array>
#include <string>
#include <iostream>
#include <math.h>
#include <std_msgs/String.h>

namespace hare{

  //put enums and other things here
  //maybe add differentiation between ugv and uav here
  enum EntityType {ROBOT, OBJECT, OBSTACLE, UNKNOWN};
  enum ObstacleType {WALL, RAMP, BOX, TUNNEL_BOX, SPHERE, CYLINDER, TUNNEL_CYLINDER, CONE};
  enum TerrainType {INDOOR_GROUND, OUTDOOR_GROUND, AIR, WATER, WATER_SURFACE};

  //Comms stuff
  enum SpinnerType {SINGLE, MULTITHREADED, ASYNCRONOUS};

  //custom messages
  ROS_DECLARE_MESSAGE(Obstacle);
  ROS_DECLARE_MESSAGE(HareUpdate);


}

#endif /* COMMON_INCLUDES_H */
