#ifndef MAP_H
#define MAP_H

#include "common_includes.h"
#include "utility.h"
#include "HareMap.h"
#include <hare/cell.h>
#include <string>
#include <fstream>
#include <hare/HareUpdate.h>


namespace hare{
  class Map{

    std::string ns;//parent namespace

    map_node knownMap[MAP_X][MAP_Y];

    std::vector<pq_node> frontier;
    std::vector<pq_node> from;

  public:

    //parent namespace
    Map(std::string ns);
    ~Map();

    // initialize the map
    void initializeMap();

    void setNamespace(std::string ns);

    // updates the map at each timestep or tick or whatever

    //WARNING float2 update is the only methods that transforms location using ODOM_TO_MAP
    void update(float2 location, int characteristic);
    void update(int2 location, int characteristic);
    void update(int2 location, const map_node& _node);
    void update(const int4& minMax, const std::vector<map_node>& region);
    void update(const hare::cell &_cell);

    // for testing and such
    // path includes name
    void saveAsString(std::string path);

    // A* algorithm
    // returns linear spline path
    std::vector<pq_node> getPath(uint8_t* capabilities, float2 _start, float2 _goal);


  private:

    // float
    float euclid(float2 a, float2 b);

    // insert sorted by priority
    void insert_pq(pq_node n, float h);

    // grabs neightbors of node n
    std::vector<pq_node> getNeighbors(pq_node n);

    // see if a node is in the priority queue
    bool isIn(pq_node n);

  };
}

#endif /* MAP_H */














// yee
