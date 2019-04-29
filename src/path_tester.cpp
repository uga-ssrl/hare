
#include "common_includes.h"
#include "Entity.h"
#include "Map.h"
#include <sstream>
#include <fstream>
#include <string>


int main(int argc, char **argv){
  ros::init(argc, argv, "hare_tester_node");

  std::cout << "testing path planning... " << std::endl;
  // do some tests here
  int4 map_truth[MAP_X][MAP_Y];
  // make the map
  hare::Map m = hare::Map("");

  // initialize the map
  m.initializeMap();
  std::cout << "map initialized..." << std::endl;

  // save as string
  m.saveAsString("/home/parallels/Development/VM_Dev/catkin_ws/src/hare/util/test1.raw");
  std::cout << "test 1 saved..." << std::endl;

  // ======================================================================== //
  // housekeeping to read in map
  std::string line;
  std::ifstream mapfile ("/home/parallels/Development/VM_Dev/catkin_ws/src/hare/util/hare.cpp.map");
  // mapfile.open();
  if (mapfile.is_open()){
    std::cout << "mapfile opened..." << std::endl;
    int linecount = 0;
    while (getline(mapfile,line)){
      // std::cout << "grabbed line ..." << std::endl;
      // std::cout << line << '\n';
      int i;
      std::stringstream ss(line);
      std::vector<int> vect;
      while (ss >> i) {
          vect.push_back(i);
          if (ss.peek() == ',')
              ss.ignore();
      }
    }
    mapfile.close();
  }
  else {
    std::cout << "Unable to open mapfile" << std::endl << " ======== NOTE ========" << std::endl;
    std::cout << ">> Make sure the change the absolute path" << std::endl;
    return 0;
  }
  // ======================================================================== //

  std::cout << "initialized map truth" << std::endl;




  return 0;
}
