
#include "common_includes.h"
#include "utility.h"
#include "Entity.h"
#include "Map.h"
#include "HareMap.h"
#include <sstream>
#include <fstream>
#include <string>


int main(int argc, char **argv){
  ros::init(argc, argv, "hare_tester_node");

  std::cout << "testing path planning... " << std::endl;
  // do some tests here
  // int4 map_truth[MAP_X][MAP_Y];
  // make the map

  // ======================================================================== //
  // test 1
  hare::Map m = hare::Map("");

  // initialize the map
  m.initializeMap();
  std::cout << "map initialized..." << std::endl;

  // save as string
  m.saveAsString("./src/hare/util/test1.raw");


  // ======================================================================== //
  // test 2
  // reset
  m.initializeMap();
  // hare::map_node asdf = hare::fullMap[1][5];
  for (int i = 0; i < 200; i++){
    for (int j = 0; j < 200; j++){
      int2 loc = {i,j};
      // std::cout <<  hare::fullMap[i][j].characteristic << "\t";
      m.update(loc, hare::fullMap[i][j]);
    }
  }

  // see if we have filled the map
  m.saveAsString("./src/hare/util/test2.raw");

  // ======================================================================== //
  // test 3
  // reset
  m.initializeMap();
  // hare::map_node asdf = hare::fullMap[1][5];
  for (int i = 50; i < 150; i++){
    for (int j = 50; j < 150; j++){
      int2 loc = {i,j};
      // std::cout <<  hare::fullMap[i][j].characteristic << "\t";
      m.update(loc, hare::fullMap[i][j]);
    }
  }

  // see if we have filled the map
  m.saveAsString("./src/hare/util/test3.raw");


  // ======================================================================== //
  // test 4
  // reset
  m.initializeMap();
  // hare::map_node asdf = hare::fullMap[1][5];
  for (int i = 50; i < 150; i++){
    for (int j = 50; j < 150; j++){
      int2 loc = {i,j};
      // std::cout <<  hare::fullMap[i][j].characteristic << "\t";
      m.update(loc, hare::fullMap[i][j]);
    }
  }

  int2 loc = {125,70};
  int2 goal = {90,115};

  std::vector<hare::pq_node> path = m.getPath(loc,goal);
  std::cout << "path length: " << path.size() << std::endl;

  hare::map_node path_guy;
  path_guy.terrain = 10;
  path_guy.explored = true;
  path_guy.traversable = true;
  path_guy.walls = {10,10,10,10};

  // fill path
  for (int i = 0; i < path.size(); i++){
    int2 loc = {path[i].x, path[i].y};
    m.update(loc,path_guy);
  }

  // see if we have filled the map
  m.saveAsString("./src/hare/util/test4.raw");


  return 0;
}
