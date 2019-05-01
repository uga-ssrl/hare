
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

  std::cout << "=========================================" << std::endl;
  std::cout << "testing path planning... " << std::endl;
  // do some tests here
  // int4 map_truth[MAP_X][MAP_Y];
  // make the map

  // ======================================================================== //
  // test 1
  hare::Map m = hare::Map("");

  // initialize the map
  m.initializeMap();
  std::cout << "[Test 1] map initialized..." << std::endl;

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
      // std::cout <<  hare::fullMap[i][j].terrain << "\t";
      m.update_debug(loc, hare::fullMap[i][j]);
    }
  }

  // see if we have filled the map
  std::cout << "[Test 2] map filled..." << std::endl;
  m.saveAsString("./src/hare/util/test2.raw");

  // ======================================================================== //
  // test 3
  // reset
  m.initializeMap();
  // hare::map_node asdf = hare::fullMap[1][5];
  for (int i = 50; i < 150; i++){
    for (int j = 50; j < 150; j++){
      int2 loc = {i,j};
      // std::cout <<  hare::fullMap[i][j].terrain << "\t";
      m.update_debug(loc, hare::fullMap[i][j]);
    }
  }

  // see if we have filled the map
  std::cout << "[Test 3] map partially filled..." << std::endl;
  m.saveAsString("./src/hare/util/test3.raw");


  // ======================================================================== //
  // // test 4
  // reset
  m.initializeMap();
  // hare::map_node asdf = hare::fullMap[1][5];
  for (int i = 50; i < 150; i++){
    for (int j = 50; j < 150; j++){
      int2 loc = {i,j};
      // std::cout <<  hare::fullMap[i][j].terrain << "\t";
      m.update_debug(loc, hare::fullMap[i][j]);
    }
  }

  int2 loc = {MAP_X/2,MAP_Y/2};
  int2 goal = {MAP_X,MAP_Y};

  std::vector<hare::pq_node> path = m.getPath(loc,goal);
  std::cout << "[Test 4] path length: " << path.size() << std::endl;

  hare::map_node path_guy;
  path_guy.terrain = 10;
  path_guy.explored = true;
  path_guy.traversable = true;
  path_guy.walls = {10,10,10,10};

  // fill path
  for (int i = 0; i < path.size(); i++){
    int2 loc = {path[i].x, path[i].y};
    m.update_debug(loc,path_guy);
  }

  // see if we have filled the map
  std::cout << "[Test 4] simple path planned..." << std::endl;
  std::cout << "[Test 4] starts at: (" << path[0].x << "," << path[0].y << ")" << std::endl;
  std::cout << "[Test 4] ends at: (" << path[path.size()-1].x << "," << path[path.size()-1].y << ")" << std::endl;

  m.saveAsString("./src/hare/util/test4.raw");


  // ======================================================================== //
  // // test 5
  // reset
  m.initializeMap();
  // hare::map_node asdf = hare::fullMap[1][5];
  for (int i = 50; i < 150; i++){
    for (int j = 50; j < 150; j++){
      int2 loc = {i,j};
      // std::cout <<  hare::fullMap[i][j].terrain << "\t";
      m.update_debug(loc, hare::fullMap[i][j]);
    }
  }

  int2 loc_5 = {125,70};
  // int2 goal = {90,115};

  hare::pq_node pq_boi = m.getNextToExplore(loc_5);
  std::vector<hare::pq_node> path_5 = m.getPath(loc_5,{pq_boi.x,pq_boi.y});
  std::cout << "[Test 5] path length: " << path_5.size() << std::endl;

  hare::map_node path_guy_5;
  path_guy_5.terrain = 10;
  path_guy_5.explored = true;
  path_guy_5.traversable = true;
  path_guy_5.walls = {10,10,10,10};

  // fill path
  for (int i = 0; i < path_5.size(); i++){
    int2 loc = {path_5[i].x, path_5[i].y};
    m.update_debug(loc,path_guy_5);
  }

  // see if we have filled the map
  std::cout << "[Test 5] simple path planned..." << std::endl;
  std::cout << "[Test 5] starts at: (" << path_5[0].x << "," << path_5[0].y << ")" << std::endl;
  std::cout << "[Test 5] ends at: (" << path_5[path_5.size()-1].x << "," << path_5[path_5.size()-1].y << ")" << std::endl;

  m.saveAsString("./src/hare/util/test5.raw");



  // ======================================================================== //
  std::cout << ">> DONE!"<< std::endl;

  return 0;
}
