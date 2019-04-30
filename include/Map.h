#ifndef MAP_H
#define MAP_H

#include "common_includes.h"
#include "utility.h"
#include "HareMap.h"
#include <hare/cell.h>
#include <string>
#include <queue>
#include <fstream>
#include <hare/HareUpdate.h>


namespace hare{
  class Map{

    std::vector<int> parentTerrain;
    std::string ns;//parent namespace

    std::vector<pq_node> frontier;
    std::vector<pq_node> from;

    // std::vector<int2> i2_path;
    // std::queue<vector<int> > queue;
    // std::vector<int2> i2_frontier;

  public:

    map_node knownMap[MAP_X][MAP_Y];

    Map();
    Map(std::string ns);
    Map(std::vector<int> parentTerrain);
    Map(std::string ns, std::vector<int> parentTerrain);
    ~Map();

    // initialize the map
    void initializeMap();

    void setNamespace(std::string ns);
    void setParentTerrain(std::vector<int> parentTerrain);

    // updates the map at each timestep or tick or whatever

    void update(int2 location, int terrain);
    void update_debug(int2 location, int terrain);
    void update(int2 location, map_node& node);
    void update_debug(int2 location, map_node& node);
    void update(const int4& minMax, const std::vector<map_node>& region);
    void update(const hare::cell &_cell);

    // for testing and such
    // path includes name
    void saveAsString(std::string path);

    // A* algorithm
    // returns linear spline path
    std::vector<pq_node> getPath(int2 start, int2 goal);


  private:

    // insert sorted by priority
    void insert_pq(pq_node n);

    // grabs neightbors of node n
    std::vector<pq_node> getNeighbors(pq_node n);

    // see if a node is in the priority queue
    bool isIn(pq_node n);

  };
}

#endif /* MAP_H */














// yee
