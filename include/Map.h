#ifndef MAP_H
#define MAP_H

#include "common_includes.h"
#include "utility.h"

// Map size
#define MAP_X 200
#define MAP_Y 200
// obstacles are entered in increments of 0.5
// here, in the descrete map of size MAX_X, each grid point represents 0.25
// so, every 4 cells in this map are really 1 cell
#define ODOM_TO_MAP 4
#define MAP_TO_ODOM 0.25

// Map Values
// see ../msg/Obstacle.msg for details about short values mapping
#define WALL 0
#define RAMP 1
#define BOX 2
#define TUNNEL_BOX 3
#define SPHERE 4
#define CYLINDER 5
#define TUNNEL_CYLINDER 6
#define CONE 7
#define SMALL_TUNNEL_ENTRANCE 8
#define LARGE_TUNNEL_ENTRANCE 9
#define RAMP_ENTRANCE 10
#define SMALL_TUNNEL_EXIT 11
#define LARGE_TUNNEL_EXIT 12
#define RAMP_EXIT 13
#define NONE 14
#define UKNOWN 15 // only for map, not used in messages


namespace hare{
  class Map{

    std::string ns;//parent namespace

    // see ../msg/Obstacle.msg for details about short values mapping
    uint8_t knownMap[MAP_X][MAP_Y];

  public:

    // let there be light
    Map();
    ~Map();

    // initialize the map
    void initializeMap();

    // updates the map at each timestep or tick or whatever
    void updateMap(float2 location, uint8_t discription);
    

  };
}

#endif /* MAP_H */


















// yee
