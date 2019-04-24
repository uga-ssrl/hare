#include "Map.h"


hare::Map::Map(){

}

hare::Map::~Map(){

}


// initialize the map
// sets all map valies to NONE
void hare::Map::initializeMap(){
  for (int i = 0; i < MAP_X; i++){
    for (int j = 0; j < MAP_Y; j++){
      knownMap[i][j] = UNKNOWN;
    }
  }
}


// update the map
void hare::Map::updateMap(float2 location, uint8_t discription){
  int2 insert;
  insert.x = (int) (ODOM_TO_MAP * location.x);
  insert.y = (int) (ODOM_TO_MAP * location.y);
  knownMap[insert.x][insert.y] = discription;
}
