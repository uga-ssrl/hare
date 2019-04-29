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
      knownMap[i][j].characteristic = UNKNOWN;
      knownMap[i][j].explored = false;
      knownMap[i][j].traversable = false; //only valid if expored is true
    }
  }
}

// update the map
void hare::Map::updateMap(float2 location, int description){
  int2 insert;
  insert.x = (int) (ODOM_TO_MAP * location.x);
  insert.y = (int) (ODOM_TO_MAP * location.y);
  knownMap[insert.x][insert.y].walls = {description,description,description,description};
  knownMap[insert.x][insert.y].explored = true;
  // TODO make sure robots can only traverse what they really can
  knownMap[insert.x][insert.y].traversable = true; //check if traversable
}

// update the map
void hare::Map::updateMap(int2 insert, int description) {
  knownMap[insert.x][insert.y].explored = true;
  knownMap[insert.x][insert.y].characteristic = description;
  // TODO make sure robots can only traverse what they really can
  knownMap[insert.x][insert.y].traversable = true; //check if traversable
}

// use this boi
void hare::Map::updateMap(int2 location, hare::map_node* mnode){
  knownMap[location.x][location.y] = *mnode;
}

void hare::Map::updateMap(hare::cellConstPtr cell){
  int2 loc = {cell->x,cell->y};
  knownMap[loc.x][loc.y].explored = cell->explored;
  knownMap[loc.x][loc.y].traversable = cell->traversable;
  knownMap[loc.x][loc.y].walls = {cell->wallLeft,cell->wallUp,cell->wallDown,cell->wallRight};
}

void hare::Map::updateMap(std::vector<hare::cellConstPtr> cells){
  for(auto cell = cells.begin(); cell != cells.end(); ++cell){
    int2 loc = {(*cell)->x,(*cell)->y};
    knownMap[loc.x][loc.y].explored = (*cell)->explored;
    knownMap[loc.x][loc.y].traversable = (*cell)->traversable;
    knownMap[loc.x][loc.y].walls = {(*cell)->wallLeft,(*cell)->wallUp,(*cell)->wallDown,(*cell)->wallRight};
  }
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
  for (int i = 1; i < (MAP_X - 1); i++){
    for (int j = 1; j < (MAP_Y - 1); j++){
      if (knownMap[i][j].explored){
        myfile << knownMap[i][j].characteristic;
        if (j < (MAP_Y - 2)) myfile << ",";
      } else {
        myfile << "-2";
        if (j < (MAP_Y - 2)) myfile << ",";
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
std::vector<hare::pq_node> hare::Map::getPath(uint8_t* capabilities, float2 _start, float2 _goal){
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
  insert_pq(curr, 0.0);
  std::vector<pq_node>  path;

  while (frontier.size()){ // while frontier not empty
    curr = frontier.back();
    frontier.pop_back();
    if (curr.x == goal.x && curr.y == goal.y){
      // TODO insure correct path
      break;
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
  return sqrt((a.x - b.x)*(a.x - b.x)  + (a.y - b.y)*(a.y - b.y));
}


// insert into priority queue
void hare::Map::insert_pq(hare::pq_node n, float h){
  pq_node elem;
  int pos = -1;
  // find where to insert
  for (int i = 0; i < frontier.size(); i++){
     elem = frontier.at(i);
    if (n.h <= elem.h){
      auto it = frontier.insert(frontier.begin() + i, n);
      break;
    }
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
