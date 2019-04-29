
#include "common_includes.h"
#include "Entity.h"
#include "Map.h"
#include <fstream>


int main(int argc, char **argv){
  ros::init(argc, argv, "hare_tester_node");

  std::cout << "testing path planning... " << std::endl;
  // do some tests here


  // housekeeping to read in map
  


  // make the map
  hare::Map m = hare::Map("");

  // initialize the map
  m.initializeMap();


  return 0;
}
