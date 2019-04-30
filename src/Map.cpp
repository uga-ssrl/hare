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
      //
      this->h_map[i][j] = 9999999; // ¯\_(ツ)_/¯ // TODO SHOULD MAKE IT MAX INT
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

//update the map
void hare::Map::update_debug(int2 location, int terrain) {
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

void hare::Map::update_debug(int2 location, map_node& node){
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

  // std::cout << "getting path..." << std::endl;

  // set all path helpers false
  for (int i = 0; i < (MAP_X); i++){
    for (int j = 0; j < (MAP_Y); j++){
      knownMap[i][j].path_helper = false;
    }
  }

  // we are at the start cell
  knownMap[start.x][start.y].path_helper = true;

  // the q boi
  std::queue<hare::pq_node> q;
  // std::cout << "made q..." << std::endl;

  hare::pq_node pq_n = {start.x, start.y, 0.0};
  q.push(pq_n);
  // std::cout << "added to q..." << std::endl;


  while (!q.empty()){
    // std::cout << "main loop..." << std::endl;
    hare::pq_node curr = q.front();
    int2 loc = {curr.x,curr.y};

    // we're good
    if (loc.x == goal.x && loc.y == goal.y) break;

    // Before pop, store it
    h_map[curr.x][curr.y] = curr.h;
    q.pop();

    // check neighbors
    if (loc.x + 1 < MAP_X){
      int2 test = {loc.x + 1,loc.y};
      if (isValid(test) && !knownMap[test.x][test.y].path_helper){
        knownMap[test.x][test.y].path_helper = true;
        hare::pq_node adj = {test.x,test.y, curr.h + 1.0};
        q.push(adj);
      }
    }
    if (loc.y + 1 < MAP_Y){
      int2 test = {loc.x,loc.y+1};
      if (isValid(test) && !knownMap[test.x][test.y].path_helper){
        knownMap[test.x][test.y].path_helper = true;
        hare::pq_node adj = {test.x,test.y, curr.h + 1.0};
        q.push(adj);
      }
    }
    if (loc.x - 1 > 0) {
      int2 test = {loc.x - 1,loc.y};
      if (isValid(test) && !knownMap[test.x][test.y].path_helper){
        knownMap[test.x][test.y].path_helper = true;
        hare::pq_node adj = {test.x,test.y, curr.h + 1.0};
        q.push(adj);
      }
    }
    if (loc.y - 1 > 0) {
      int2 test = {loc.x,loc.y - 1};
      if (isValid(test) && !knownMap[test.x][test.y].path_helper){
        knownMap[test.x][test.y].path_helper = true;
        hare::pq_node adj = {test.x,test.y, curr.h + 1.0};
        q.push(adj);
      }
    }
  }

  std::vector<hare::pq_node> temp;
  // get the goal node:
  hare::pq_node pos = q.front();
  int itr = (int) pos.h;
  temp.push_back(pos);
  for (int i = 0; i < itr; i++){
    if (h_map[pos.x-1][pos.y] < pos.h){
      pos = {pos.x-1,pos.y,pos.h-1};
    }
    else if (h_map[pos.x+1][pos.y] < pos.h){
      pos = {pos.x+1,pos.y,pos.h-1};
    }
    else if (h_map[pos.x][pos.y-1] < pos.h){
      pos = {pos.x,pos.y-1,pos.h-1};
    }
    else if (h_map[pos.x][pos.y+1] < pos.h){
      pos = {pos.x,pos.y+1,pos.h-1};
    }
    temp.push_back(pos);
  }


  return temp;
}


hare::pq_node hare::Map::getNextToExplore(int2 start){

  // find closest unexplored
  hare::pq_node min = {-1,-1,990000};

  for (int i = 0; i < (MAP_X); i++){
    for (int j = 0; j < (MAP_Y); j++){
      // hare::map_node map_test = knownMap[i][j];
      if (isValid({i,j}) && !knownMap[i][j].explored && manhattan({i,j},start) <= min.h){
        min = {i,j,manhattan({i,j},start)};
      }
    }
  }

  return min;
}

int hare::Map::manhattan(int2 a, int2 b){
  return (abs(a.x - b.x) + abs(a.y - b.y));
}

bool hare::Map::isValid(int2 n){
  // TODO see what the robot's stuff can do here
  // only valid if the robot
  if (hare::fullMap[n.x][n.y].terrain == 0 || hare::fullMap[n.x][n.y].terrain == 2){
    return true;
  }
  return false;
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
