#include "Map.h"

hare::Map::Map(){
  this->initializeMap();
}
hare::Map::Map(std::string ns){
  this->ns = ns;
  this->initializeMap();
}
hare::Map::Map(std::vector<int> parentTerrain){
  this->parentTerrain = parentTerrain;
  this->initializeMap();
}
hare::Map::Map(std::string ns, std::vector<int> parentTerrain){
  this->ns = ns;
  this->parentTerrain = parentTerrain;
  this->initializeMap();
}
hare::Map::~Map(){

}


// initialize the map
// sets all map valies to NONE
void hare::Map::initializeMap(){
  for (int i = 0; i < MAP_X; i++){
    for (int j = 0; j < MAP_Y; j++){
      this->knownMap[i][j].terrain = -1;
      this->knownMap[i][j].explored = false;
      this->knownMap[i][j].traversable = false; //only valid if expored is true
      this->knownMap[i][j].walls = {-1,-1,-1,-1};
    }
  }
}

//update the map
void hare::Map::update(int2 location, int terrain) {
  if(this->parentTerrain.size() == 0){
    ROS_ERROR("PARENT TERRAIN IN MAP MUST BE SET BEFORE UPDATING");
  }
  this->knownMap[location.x][location.y].explored = true;
  this->knownMap[location.x][location.y].terrain = terrain;
  this->knownMap[location.x][location.y].walls = {terrain,terrain,terrain,terrain};
  this->knownMap[location.x][location.y].traversable =
  std::find(this->parentTerrain.begin(),this->parentTerrain.end(),
  terrain) != this->parentTerrain.end();
}
void hare::Map::update(int2 location, map_node& node){
  if(this->parentTerrain.size() == 0){
    ROS_ERROR("PARENT TERRAIN IN MAP MUST BE SET BEFORE UPDATING");
  }
  node.traversable = std::find(this->parentTerrain.begin(),this->parentTerrain.end(), node.terrain) != this->parentTerrain.end();
  this->knownMap[location.x][location.y] = node;
}
void hare::Map::update(const int4& minMax, const std::vector<hare::map_node>& region){
  int currentElement = 0;
  if(this->parentTerrain.size() == 0){
    ROS_ERROR("PARENT TERRAIN IN MAP MUST BE SET BEFORE UPDATING");
  }
  for(int x = minMax.x; x < minMax.z; ++x){
    for(int y = minMax.y; y < minMax.w; ++y){
      this->knownMap[x][y] = region[currentElement];
      this->knownMap[x][y].traversable =
      std::find(this->parentTerrain.begin(),this->parentTerrain.end(),
      region[currentElement].terrain) != this->parentTerrain.end();
      currentElement++;
    }
  }
}
void hare::Map::update(const hare::cell &_cell){
  if(this->parentTerrain.size() == 0){
    ROS_ERROR("PARENT TERRAIN IN MAP MUST BE SET BEFORE UPDATING");
  }
  int2 location = {_cell.x,_cell.y};
  this->knownMap[location.x][location.y].traversable =
  std::find(this->parentTerrain.begin(),this->parentTerrain.end(),
  _cell.terrain) != this->parentTerrain.end();
  this->knownMap[location.x][location.y].explored = _cell.explored;
  this->knownMap[location.x][location.y].walls = {_cell.wallLeft,_cell.wallUp,
    _cell.wallDown,_cell.wallRight};
}

// set the robot
void hare::Map::setNamespace(std::string ns){
  this->ns = ns;
}
void hare::Map::setParentTerrain(std::vector<int> parentTerrain){
  this->parentTerrain = parentTerrain;
}

// save the map file as an image!
void hare::Map::saveAsString(std::string path){
  std::ofstream myfile;
  myfile.open (path);
  // start at one, end after 1
  for (int i = 0; i < (MAP_X); i++){
    for (int j = 0; j < (MAP_Y); j++){
      if (knownMap[i][j].explored){
        myfile << knownMap[i][j].terrain;
        if (j < (MAP_Y - 1)) myfile << ",";
      } else {
        myfile << "-2";
        if (j < (MAP_Y - 1)) myfile << ",";
      }
    }
    myfile << "\n";
  }
}

//NOTE THIS IS VERY INEFFICIENT AS IT IS - WORKS THOUGH
// Greedy heuristic algorithm
// returns linear spline path
// We assume only up, down, left, right movements
// https://www.redblobgames.com/pathfinding/a-star/introduction.html#greedy-best-first
// description is the same as capabilities
std::vector<hare::pq_node> hare::Map::getPath(int2 start, int2 goal){
  std::cout << "entered getPath... " << std::endl;
  pq_node curr;
  curr.x = start.x;
  curr.y = start.y;
  curr.h = 0.0;
  // the structs to be used
  insert_pq(curr);
  // std::vector<pq_node> path;

  std::cout << "initialized the stuff... " << std::endl;
  std::cout << "frontier size: " << frontier.size() << std::endl;
  while (frontier.size()){ // while frontier not empty
    curr = frontier.back();
    frontier.pop_back();

    if (curr.x == goal.x && curr.y == goal.y){
      // TODO insure correct path
      return from;
    }

    std::vector<pq_node> neighbors;
    neighbors = getNeighbors(curr);

    for (int i = 0; i < neighbors.size(); i++){
      pq_node next = neighbors[i];
      if ( !isIn(next) ) {
        float2 a;
        a.x = next.x;
        a.y = next.y;
        float h = euclid(a,{(float)goal.x,(float)goal.y});
        next.h = h;
        insert_pq(next);
        from.push_back(curr);
      }
    }
  }

  return from;
}

// just euclidian distance
float hare::Map::euclid(float2 a, float2 b){
  return abs((a.x - b.x)  + (a.y - b.y));
}

// insert into priority queue
void hare::Map::insert_pq(hare::pq_node n){
  if (this->frontier.size()) {
    int i = 0;
    while (this->frontier[i].h > n.h && i < this->frontier.size()) i++;
    this->frontier.insert(this->frontier.begin()+i, n);
  }
  else {
    this->frontier.insert(this->frontier.begin(), n);
  }
}

// get the neighbors of a pq_node
std::vector<hare::pq_node> hare::Map::getNeighbors(hare::pq_node n){
  std::vector<pq_node> neighbors;
  // TODO potentially use explored?
  if (this->knownMap[n.x][n.y + 1].traversable) {
    pq_node new_guy;
    new_guy.x = n.x;
    new_guy.y = n.y + 1;
    new_guy.h = -1.0; // needs to be updated later
    neighbors.push_back(new_guy);
  }
  if (this->knownMap[n.x + 1][n.y].traversable) {
    pq_node new_guy;
    new_guy.x = n.x + 1;
    new_guy.y = n.y;
    new_guy.h = -1.0; // needs to be updated later
    neighbors.push_back(new_guy);
  }
  if (this->knownMap[n.x][n.y - 1].traversable) {
    pq_node new_guy;
    new_guy.x = n.x;
    new_guy.y = n.y - 1;
    new_guy.h = -1.0; // needs to be updated later
    neighbors.push_back(new_guy);
  }
  if (this->knownMap[n.x - 1][n.y].traversable) {
    pq_node new_guy;
    new_guy.x = n.x - 1;
    new_guy.y = n.y;
    new_guy.h = -1.0; // needs to be updated later
    neighbors.push_back(new_guy);
  }
  return neighbors;
}

// see if n is in from
bool hare::Map::isIn(hare::pq_node n){
  for (int i = 0; i < from.size(); i++){
    if (this->from[i].x == n.x && this->from[i].y == n.y){ return true; }
  }
  return false;
}
