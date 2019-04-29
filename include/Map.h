#ifndef MAP_H
#define MAP_H

#include "common_includes.h"
#include "utility.h"
#include "HareMap.h"
#include <hare/cell.h>
<<<<<<< HEAD
#include <string>
#include <fstream>
=======
#include <hare/HareUpdate.h>
>>>>>>> 9d1db657130b0e1543a3c6e09de9e1b170036665


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

    void updateMap(float2 location, int description);
    void updateMap(int2 insert, int description);
    void updateMap(int2 location, map_node* mnode);
    void updateMap(cellConstPtr cell);
    void updateMap(std::vector<cellConstPtr> cells);

    void update(int2 location, int description);
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
