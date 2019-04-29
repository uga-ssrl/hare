#include "Map.h"

hare::Map::Map(std::string ns){
  this->ns = ns;
}

hare::Map::~Map(){

}


// initialize the map
// sets all map valies to NONE
void hare::Map::initializeMap(){
  for (int i = 0; i < MAP_X; i++){
    for (int j = 0; j < MAP_Y; j++){
      this->knownMap[i][j].characteristic = UNKNOWN;
      this->knownMap[i][j].explored = false;
      this->knownMap[i][j].traversable = false; //only valid if expored is true
    }
  }
}


//WARNING THIS ASSUMES THAT ODOM_TO_MAP tf is needed
// update the map
void hare::Map::update(float2 location, int characteristic){
  int2 insert;
  insert.x = (int) (ODOM_TO_MAP * location.x);
  insert.y = (int) (ODOM_TO_MAP * location.y);
  this->knownMap[insert.x][insert.y].walls = {characteristic,characteristic,characteristic,characteristic};
  this->knownMap[insert.x][insert.y].characteristic = characteristic;
  this->knownMap[insert.x][insert.y].explored = true;
  // TODO make sure robots can only traverse what they really can
  this->knownMap[insert.x][insert.y].traversable = true; //check if traversable
}

// update the map
void hare::Map::update(int2 location, int characteristic) {
  this->knownMap[location.x][location.y].explored = true;
  this->knownMap[location.x][location.y].characteristic = characteristic;
  this->knownMap[location.x][location.y].walls = {characteristic,characteristic,characteristic,characteristic};
  // TODO make sure robots can only traverse what they really can
  int bot = 1;
  if (bot == 1){ // only done for clarity
    // big robot test
    int guy = knownMap[location.x][location.y].characteristic;
    if (guy == 0) {
      this->knownMap[location.x][location.y].traversable = true; //check if traversable
    }
    else this->knownMap[location.x][location.y].traversable = false;
  } else if (bot == 2) {
    // something else
  } else if (bot == 3) {
    // something else
  }
}

void hare::Map::update(int2 location, map_node _node){
  knownMap[location.x][location.y] = _node;
}

void hare::Map::update(const int4& minMax, const std::vector<hare::map_node>& region){
  int currentElement = 0;
  for(int x = minMax.x; x < minMax.z; ++x){
    for(int y = minMax.y; y < minMax.w; ++y){
      this->knownMap[x][y] = region[currentElement++];
    }
  }
}

void hare::Map::update(const hare::cell &_cell){
  int2 location = {_cell.x,_cell.y};
  this->knownMap[location.x][location.y].traversable = _cell.traversable;
  this->knownMap[location.x][location.y].explored = _cell.explored;
  this->knownMap[location.x][location.y].walls = {_cell.wallLeft,_cell.wallUp,
    _cell.wallDown,_cell.wallRight};
}

// set the robot
void hare::Map::setNamespace(std::string ns){
  this->ns = ns;
}

// save the map file as an image!
void hare::Map::saveAsString(std::string path){
  std::ofstream myfile;
  myfile.open (path);
  // start at one, end after 1
  for (int i = 0; i < (MAP_X); i++){
    for (int j = 0; j < (MAP_Y); j++){
      if (knownMap[i][j].explored){
        myfile << knownMap[i][j].characteristic;
        if (j < (MAP_Y - 1)) myfile << ",";
      } else {
        myfile << "-2";
        if (j < (MAP_Y - 1)) myfile << ",";
      }
    }
    myfile << "\n";
  }
}


// Greedy heuristic algorithm
// returns linear spline path
// We assume only up, down, left, right movements
// https://www.redblobgames.com/pathfinding/a-star/introduction.html#greedy-best-first
// description is the same as capabilities
std::vector<hare::pq_node> hare::Map::getPath(float2 _start, float2 _goal){
  std::cout << "entered getPath... " << std::endl;
  int2 goal;
  int2 start;
  goal.x  = (int) (ODOM_TO_MAP * _goal.x);
  goal.y  = (int) (ODOM_TO_MAP * _goal.y);
  start.x = (int) (ODOM_TO_MAP * _start.x);
  start.y = (int) (ODOM_TO_MAP * _start.y);
  pq_node curr;
  curr.x = start.x;
  curr.y = start.y;
  curr.h = 0.0;
  // the structs to be used
  insert_pq(curr, curr.h);
  // std::vector<pq_node> path;

  std::cout << "initialized the stuff... " << std::endl;
  std::cout << "frontier size: " << frontier.size() << std::endl;
  while (frontier.size()){ // while frontier not empty
    // std::cout << "doing it... " << std::endl;
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
        float h = euclid(a,_goal);
        next.h = h;
        insert_pq(next, next.h);
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
void hare::Map::insert_pq(hare::pq_node n, float h){
  if (frontier.size()) {
    // std::cout << "ok" << "\t";
    int i = 0;
    while (frontier[i].h > n.h && i < frontier.size()) i++;
    auto it = frontier.insert(frontier.begin()+i, n);
  } else {
    auto it = frontier.insert(frontier.begin(), n);
  }
}

// get the neighbors of a pq_node
std::vector<hare::pq_node> hare::Map::getNeighbors(hare::pq_node n){
  std::vector<pq_node> neighbors;
  // TODO potentially use explored?
  if (knownMap[n.x][n.y + 1].traversable) {
    pq_node new_guy;
    new_guy.x = n.x;
    new_guy.y = n.y + 1;
    new_guy.h = -1.0; // needs to be updated later
    neighbors.push_back(new_guy);
  }
  if (knownMap[n.x + 1][n.y].traversable) {
    pq_node new_guy;
    new_guy.x = n.x + 1;
    new_guy.y = n.y;
    new_guy.h = -1.0; // needs to be updated later
    neighbors.push_back(new_guy);
  }
  if (knownMap[n.x][n.y - 1].traversable) {
    pq_node new_guy;
    new_guy.x = n.x;
    new_guy.y = n.y - 1;
    new_guy.h = -1.0; // needs to be updated later
    neighbors.push_back(new_guy);
  }
  if (knownMap[n.x - 1][n.y].traversable) {
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
    if (from[i].x == n.x && from[i].y == n.y){ return true; }
  }
  return false;
}



























// yeet
