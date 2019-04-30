#ifndef HAREMAP_H
#define HAREMAP_H

// Map size
#define MAP_X 200
#define MAP_Y 200
// obstacles are entered in increments of 0.5
// here, in the descrete map of size MAX_X, each grid point represents 0.25
// so, every 4 cells in this map are really 1 cell
#define ODOM_TO_MAP 4
#define MAP_TO_ODOM 0.25


#include "utility.h"
#include "common_includes.h"

namespace hare{

  typedef struct map_node{
    int characteristic;
    bool explored;
    bool traversable;
    int4 walls;//-1 == wall, 0 == no wall, 1 == short entrance, 2 == large entrance
  } map_node;
  //walls.x = left, walls.y = up, walls.z = down, walls.w = right
  typedef struct pq_node{
    int x;
    int y;
    float h; // heuristic priority
  } pq_node;

  //full map generated in a rediculous way with a python file
  extern map_node fullMap[MAP_X][MAP_Y];
  
}

#endif /* HAREMAP_H */
