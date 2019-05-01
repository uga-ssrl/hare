#include "Entity.h"

hare::Entity::Entity(){
  this->id = -1;
  this->ns = "/";
  this->description.type = UNKNOWN;
  this->treeState = IDLE;
}
hare::Entity::Entity(std::string ns){
  this->ns = ns;
  this->id = std::stoi(this->ns.substr(this->ns.length() - 1, 1));
  this->description.type = UNKNOWN;
  this->treeState = IDLE;
}
hare::Entity::~Entity(){

}

hare::Neighbor::Neighbor(){
  this->id = -1;
  this->ns = "/";
  this->description.type = ROBOT;
  this->treeState = IDLE;
}
hare::Neighbor::Neighbor(std::string ns){
  this->ns = ns;
  this->id = std::stoi(this->ns.substr(this->ns.length() - 1, 1));
  this->description.type = ROBOT;
  this->treeState = IDLE;
}
hare::Neighbor::~Neighbor(){

}

hare::Robot::Robot(){
  this->id = -1;
  this->ns = "/";
  this->description.type = ROBOT;
  this->map = NULL;
  this->treeState = IDLE;
}
//set queue size in here
hare::Robot::Robot(ros::NodeHandle nh){
  this->nh = nh;
  this->ns = nh.getNamespace();
  this->id = std::stoi(this->ns.substr(this->ns.length() - 1, 1));
  this->description.type = ROBOT;
  float3 position;
  this->treeState = IDLE;
  this->nh.getParam("init_x", position.x);
  this->nh.getParam("init_y", position.y);
  this->nh.getParam("init_z", position.z);
  this->init_pose = {position.x,position.y};
  this->map = new Map(this->ns);
}
hare::Robot::~Robot(){
  if(this->map != NULL){
    delete this->map;
  }

}
//must add capabilities here if you add them to yaml file
// Add to publisher stack
void hare::Robot::loadCapabilties(){
  bool received = true;
  if(!this->nh.getParam("/static_characteristics" + this->ns + "/terrain", this->description.terrain)){
    ROS_ERROR("Failed to get param terrain");
    received = false;
  }
  if(received){
    ROS_INFO("Successfuly loaded terrain from yaml file");
  }
  this->map->setParentTerrain(this->description.terrain);
  for(auto neighbor = this->neighbors.begin(); neighbor != this->neighbors.end(); ++neighbor){
    if(!this->nh.getParam("/static_characteristics" + (*neighbor).ns + "/terrain", (*neighbor).description.terrain)){
      ROS_ERROR("Failed to get param terrain");
      received = false;
    }
    if(received){
      ROS_INFO("Successfuly loaded terrain from yaml file");
    }
  }
}
void hare::Robot::findNeighbors(){
  int numNeighbors = 0;
  std::string param_name;
  if (nh.searchParam("num_robots", param_name)){
    if(nh.getParam(param_name,numNeighbors)){
      numNeighbors--;
      ROS_INFO("Got numNeighbors: %d", numNeighbors);
    }
    else{
      ROS_ERROR("Failed to get param 'num_robots'");
    }
  }
  else{
    ROS_ERROR("No param 'num_robots' found in an upward search");
  }
  std::string temp;
  for(int i = 0; i < numNeighbors; ++i){
    temp = "neighbor" + std::to_string(i + 1);
    std::string neighbor_ns;
    if(nh.getParam(temp,neighbor_ns)){
      Neighbor n = Neighbor("/" + neighbor_ns);
      this->neighbors.push_back(n);
      ROS_INFO("Got param: %s", n.ns.c_str());
    }
    else{
      ROS_ERROR("Failed to get param 'neighbor%d'", i + 1);
    }
  }
}
bool hare::Robot::addPublisher(ros::Publisher &pub){
  bool success = true;
  if(!pub){
    ROS_ERROR("failed attempt to add bad publisher");
    success = false;
  }
  else{
    this->publishers.push_back(pub);
    this->publisherMap.insert(std::make_pair(pub.getTopic(), this->publishers.size() - 1));
  }

  return success;
}
bool hare::Robot::addSubscriber(ros::Subscriber &sub){
  bool success = true;
  if(!sub){
    ROS_ERROR("failed attempt to add bad subscriber");
    success = false;
  }
  else{
    this->subscribers.push_back(sub);
  }
  return success;
}

//add publishers and subscribers in these method
// EDITIBALE METHOD
void hare::Robot::initPublishers(){
  ros::Publisher twist_pub = this->nh.advertise<geometry_msgs::Twist>("goal", this->queue_size);
  this->addPublisher(twist_pub);
  ros::Publisher hareUpdate_pub = this->nh.advertise<HareUpdate>("HARE_UPDATE",this->queue_size);
  this->addPublisher(hareUpdate_pub);
  ros::Publisher cmdVel = this->nh.advertise<geometry_msgs::Twist>("cmd_vel",this->queue_size);
  this->addPublisher(cmdVel);
}
void hare::Robot::initSubscribers(){
  //single point subscriptions
  std::string odomTopic;
  std::string parameter = this->ns + "_odom";
  if(!this->nh.getParam(parameter, odomTopic)){
      ROS_ERROR("Failed to get param %s", parameter);
  }
  ros::Subscriber odom_sub = this->nh.subscribe<nav_msgs::Odometry>(odomTopic, this->queue_size, &hare::Robot::callback, this);
  this->addSubscriber(odom_sub);
  ROS_INFO("Successfully subscribed to %s",odomTopic.c_str());
  //per neighbor subscriptions
  for(auto neighbor = this->neighbors.begin(); neighbor != this->neighbors.end(); ++neighbor){
    std::string neighbor_ns = (*neighbor).ns;
    ros::Subscriber hareUpdate_sub = this->nh.subscribe<HareUpdate>(neighbor_ns + "/HARE_UPDATE", this->queue_size, &hare::Robot::callback, this);
    this->addSubscriber(hareUpdate_sub);
  }
}
void hare::Robot::setQueueSize(uint32_t queue_size){
  this->queue_size = queue_size;
}

void hare::Robot::init(){
  this->findNeighbors();
  this->loadCapabilties();
  this->initPublishers();
  this->initSubscribers();
}

void hare::Robot::callback(const std_msgs::StringConstPtr& msg){

}
void hare::Robot::callback(const nav_msgs::OdometryConstPtr& msg){
  this->odom.header = msg->header;
  this->odom.child_frame_id = msg->child_frame_id;
  this->odom.pose = msg->pose;
  this->odom.twist = msg->twist;
  float2 odomCoord = {this->odom.pose.pose.position.x,this->odom.pose.pose.position.y};
  if(!withinBounds(odomCoord)){
    this->stop();
    this->treeState = IDLE;
  }
}
void hare::Robot::callback(const hare::HareUpdateConstPtr& msg){
  //UPDATE MAP BASED ON CELLS
  for(int i = 0; i < msg->cells.size(); ++i){
    const hare::cell &data = msg->cells[i];
    this->map->update(data);
  }
  for(auto neighbor = this->neighbors.begin(); neighbor != this->neighbors.end(); ++neighbor){
    if((*neighbor).id == msg->robot_id){
      (*neighbor).odom = msg->odom;
      (*neighbor).treeState = static_cast<hare::HareTreeState>(msg->tree_state);
      (*neighbor).goal = {msg->goal_x,msg->goal_y};
      break;
    }
  }
}
void hare::Robot::callback(const geometry_msgs::TwistConstPtr& msg){

}

void hare::Robot::setCallBackQueue(ros::CallbackQueue callbackQueue){
  this->nh.setCallbackQueue(&callbackQueue);
}

void hare::Robot::sense(std::vector<hare::map_node>& region, int4 &minMax){
  if(minMax.x < 0) minMax.x = 0;
  if(minMax.z > MAP_X) minMax.z = MAP_X - 1;
  if(minMax.y < 0) minMax.y = 0;
  if(minMax.w > MAP_Y) minMax.w = MAP_Y - 1;
  for(int x = minMax.x; x < minMax.z; ++x){
    for(int y = minMax.y; y < minMax.w; ++y){
      hare::map_node currentNode = fullMap[x][y];
      region.push_back(currentNode);
    }
  }
}

void hare::Robot::goUp(float3 linear, float3 angular){
  float step = MAP_TO_ODOM;
  float2 start = {this->odom.pose.pose.position.x,this->odom.pose.pose.position.y};
  float2 end = {start.x,start.y+1};
  geometry_msgs::Twist cmdVel;
  cmdVel.angular.x = angular.x;
  cmdVel.angular.y = angular.y;
  cmdVel.angular.z = angular.z;
  this->publish<geometry_msgs::Twist>(cmdVel,"cmd_vel");
  ros::Duration(step*abs(cmdVel.linear.y)).sleep();
  stop();
}
void hare::Robot::goDown(float3 linear, float3 angular){
  float step = MAP_TO_ODOM;
  float2 start = {this->odom.pose.pose.position.x,this->odom.pose.pose.position.y};
  float2 end = {start.x,start.y-1};
  geometry_msgs::Twist cmdVel;
  cmdVel.linear.x = linear.x;
  cmdVel.linear.y = linear.y;
  cmdVel.angular.x = angular.x;
  cmdVel.angular.y = angular.y;
  this->publish<geometry_msgs::Twist>(cmdVel,"cmd_vel");
  ros::Duration(step*abs(cmdVel.linear.y)).sleep();
  stop();
}
void hare::Robot::goRight(float3 linear, float3 angular){
  // float step = MAP_TO_ODOM;
  float2 start = {this->odom.pose.pose.position.x,this->odom.pose.pose.position.y};
  float2 end = {start.x+1,start.y};
  geometry_msgs::Twist cmdVel;
  cmdVel.angular.x = angular.x;
  cmdVel.angular.y = angular.y;
  cmdVel.angular.z = angular.z;

  this->publish<geometry_msgs::Twist>(cmdVel,"cmd_vel");
  double time_step = (double)(1.57/(double)abs(cmdVel.angular.z));
  ros::Duration(time_step).sleep(); // 1.57 rad = 90 degree
  this->goUp();
}
void hare::Robot::goLeft(float3 linear, float3 angular){
  // float step = MAP_TO_ODOM;
  float2 start = {this->odom.pose.pose.position.x,this->odom.pose.pose.position.y};
  float2 end = {start.x-1,start.y};
  geometry_msgs::Twist cmdVel;
  cmdVel.angular.x = angular.x;
  cmdVel.angular.y = angular.y;
  cmdVel.angular.z = angular.z;

  this->publish<geometry_msgs::Twist>(cmdVel,"cmd_vel");
  double time_step = (double)(1.57/(double)abs(cmdVel.angular.z));
  ros::Duration(time_step).sleep(); // 1.57 rad = 90 degree
  this->goUp();
}
void hare::Robot::goDown(float rate){
  geometry_msgs::Twist cmdVel;
  cmdVel.linear.y = -1.0f*abs(rate);
  this->publish<geometry_msgs::Twist>(cmdVel,"cmd_vel");
}
void hare::Robot::goRight(float rate){
  geometry_msgs::Twist cmdVel;
  cmdVel.linear.x = abs(rate);
  this->publish<geometry_msgs::Twist>(cmdVel,"cmd_vel");
}
void hare::Robot::goLeft(float rate){
  geometry_msgs::Twist cmdVel;
  cmdVel.linear.x = -1.0f*abs(rate);
  this->publish<geometry_msgs::Twist>(cmdVel,"cmd_vel");
}
void hare::Robot::stop(){
  geometry_msgs::Twist cmdVel;
  cmdVel.linear.x = 0.0f;
  cmdVel.linear.y = 0.0f;
  cmdVel.linear.z = 0.0f;
  cmdVel.angular.x = 0.0f;
  cmdVel.angular.y = 0.0f;
  cmdVel.angular.z = 0.0f;
  this->publish<geometry_msgs::Twist>(cmdVel,"cmd_vel");
}
void hare::Robot::stop(float delay){
  geometry_msgs::Twist cmdVel;
  cmdVel.linear.x = 0.0f;
  cmdVel.linear.y = 0.0f;
  cmdVel.linear.z = 0.0f;
  cmdVel.angular.x = 0.0f;
  cmdVel.angular.y = 0.0f;
  cmdVel.angular.z = 0.0f;
  ros::Duration(delay).sleep();
  this->publish<geometry_msgs::Twist>(cmdVel,"cmd_vel");
}

void hare::Robot::addCentroidGoal(){
  float2 centroid = {0.0f,0.0f};
  for(auto neighbor = this->neighbors.begin(); neighbor != this->neighbors.end(); ++neighbor){
    centroid.x += (*neighbor).odom.pose.pose.position.x;
    centroid.y += (*neighbor).odom.pose.pose.position.y;
  }
  centroid.x /= this->neighbors.size();
  centroid.y /= this->neighbors.size();
  int2 mapVal = odomToMap(centroid);
  if(mapVal.x < 0) mapVal.x = 0;
  if(mapVal.x >= MAP_X) mapVal.x = MAP_X;
  if(mapVal.y < 0) mapVal.y = 0;
  if(mapVal.y >= MAP_Y) mapVal.y = MAP_Y;
  this->goals.push_back(mapVal);
}
void hare::Robot::addDispersionGoal(){
  float2 centroid = {0.0f,0.0f};
  for(auto neighbor = this->neighbors.begin(); neighbor != this->neighbors.end(); ++neighbor){
    centroid.x += (*neighbor).odom.pose.pose.position.x;
    centroid.y += (*neighbor).odom.pose.pose.position.y;
  }
  centroid.x /= this->neighbors.size();
  centroid.y /= this->neighbors.size();
  float2 v = {this->odom.pose.pose.position.x-centroid.x,this->odom.pose.pose.position.y-centroid.y};
  int2 mapVal = odomToMap(v);
  if(mapVal.x < 0) mapVal.x = 0;
  if(mapVal.x >= MAP_X) mapVal.x = MAP_X;
  if(mapVal.y < 0) mapVal.y = 0;
  if(mapVal.y >= MAP_Y) mapVal.y = MAP_Y;
  this->goals.push_back(mapVal);
}
void hare::Robot::search(std::vector<pq_node>& pathToGoal){
  int2 currentPosition = odomToMap({this->odom.pose.pose.position.x, this->odom.pose.pose.position.y});

  pq_node imidiateTarget = pathToGoal.back();
  pathToGoal.pop_back();

  float2 filler = {0.0f,0.0f};
  int2 vel = {imidiateTarget.x-currentPosition.x,imidiateTarget.y-currentPosition.y};
  float mag = sqrtf((vel.x*vel.x) + (vel.y*vel.y));
  this->go(mapToOdom({vel.x/mag,vel.y/mag}),filler);
}
void hare::Robot::search(std::vector<map_node>& sensedRegion, int4 minMax){
  int2 currentPosition = odomToMap({this->odom.pose.pose.position.x, this->odom.pose.pose.position.y});
  int2 weighting = {0,0};
  int i = 0;
  for(int x = minMax.x; x < minMax.z; ++x){
    for(int y = minMax.y; y < minMax.w; ++y, ++i){
      if(find(this->description.terrain.begin(), this->description.terrain.end(), sensedRegion[i].terrain) != this->description.terrain.end()){
        if(x < currentPosition.x) --weighting.x;
        else ++weighting.x;
        if(y < currentPosition.y) --weighting.y;
        else ++weighting.y;
      }
      else{
        sensedRegion[i].traversable = false;
        if(x < currentPosition.x) weighting.x += 2;
        else weighting.x -= 2;
        if(y < currentPosition.y) weighting.y += 2;
        else weighting.y -= 2;
      }
    }
  }
  while(currentPosition.x == (ODOM_TO_MAP*this->odom.pose.pose.position.x) + MAP_X/2 && currentPosition.y == (ODOM_TO_MAP*this->odom.pose.pose.position.y) + MAP_Y/2){
    if(abs(weighting.x) > abs(weighting.y)){
      weighting.x /=2;
      if(weighting.x > 0) this->goRight();
      else this->goLeft();
    }
    else{
      weighting.y /=2;
      if(weighting.y > 0) this->goUp();
      else this->goDown();
    }
  }
}

bool hare::Robot::isDone(){
  for(int r = 0; r < MAP_X; ++r){
    for(int c = 0; c < MAP_Y; ++c){
      if(!this->map->knownMap[r][c].explored){
        return false;
      }
    }
  }
  return true;
}

//TODO to record how rapid cell discovery is happening
//TODO make search use sensed area

void hare::Robot::run(){
  hare::HareUpdate update;
  update.robot_id = this->id;

  std::vector<hare::map_node> sensedRegion;

  this->treeState = SEARCH;
  bool done = false;
  unsigned int step = 0;
  int2 currentPosition = odomToMap(this->odom.pose.pose.position.x,
    this->odom.pose.pose.position.y);
  int4 sensoryBound = {0,0,0,0};//{min.x,min.y,max.x,max.y} - indices in fullMap
  float sensingRange = 2.0f;
  this->path.clear();
  this->goals.clear();
  this->addCentroidGoal();
  std::vector<pq_node> pathToGoal = this->map->getPath(currentPosition,{0,0});

  //ros::Rate r(50);//Hz

  while (ros::ok()){

    currentPosition = hare::odomToMap(this->odom.pose.pose.position.x,
      this->odom.pose.pose.position.y);

    //MAKE SURE TO TRANSLATE THIS POSITION TO OUR COORDINATE FRAME
    update.odom = this->odom;
    sensoryBound = {floor(currentPosition.x-sensingRange),
      floor(currentPosition.y-sensingRange),
      floor(currentPosition.x+sensingRange),
      floor(currentPosition.y+sensingRange)
    };
    sensedRegion.clear();
    this->sense(sensedRegion,sensoryBound);
    this->map->update(sensoryBound,sensedRegion);
    this->path.push_back(currentPosition);

    update.cells.clear();
    for(int x = sensoryBound.x, i = 0; x < sensoryBound.z; ++x){
      for(int y = sensoryBound.y; y < sensoryBound.w; ++y){
        hare::cell _cell;
        _cell.x = x;
        _cell.y = y;
        _cell.terrain = sensedRegion[i].terrain;
        _cell.traversable = sensedRegion[i].traversable;
        _cell.explored = sensedRegion[i].explored;
        _cell.wallLeft = sensedRegion[i].walls.x;
        _cell.wallUp = sensedRegion[i].walls.y;
        _cell.wallDown = sensedRegion[i].walls.z;
        _cell.wallRight = sensedRegion[i].walls.w;
        update.cells.push_back(_cell);
        ++i;
      }
    }
    update.tree_state = this->treeState;
    if(this->goals.size()){
      update.goal_x = this->goals.front().x;
      update.goal_y = this->goals.front().y;
    }
    else{
      update.goal_x = -1;
      update.goal_y = -1;
    }


    ///send data
    this->publish<hare::HareUpdate>(update,"HARE_UPDATE");
    std::vector<hare::pq_node> action;
    int2 goal = {4,4}; // Map goal
    bool moved = false;

    action = this->map->getPath(currentPosition,goal);

    for(auto &position : action)
    {
      int2 m_pose = {position.x, position.y};
      if (moved) break;

      if (m_pose.x < currentPosition.x)
      {
        this->goLeft();
        moved = true;
      }
      else if (m_pose.y < currentPosition.y)
      {
        this->goDown(); // -y (cmd vel) moves up in map
        moved = true;
      }
      else if (m_pose.x > currentPosition.x)
      {
        this->goRight();
        moved = true;
      }
      else if (m_pose.y > currentPosition.y)
      {
        this->goUp(); // +y (cmd vel) moves down in map
        moved = true;
      }

        // switch(this->treeState){
        //   case IDLE:{//something is wrong
        //     if(this->map->knownMap[currentPosition.y + 1][currentPosition.x].terrain == -1)
        //     {
        //       this->goLeft();
        //       if(step == 20) this->treeState = SEARCH;
        //     }
        //     break;
        //   }
        //   case SEARCH:{//simple
        //     if(this->map->knownMap[currentPosition.y + 1][currentPosition.x].terrain == -1)
        //     {
        //       this->goRight();
        //       if(step == 20) this->treeState = IDLE;
        //     }
        //
        //     break;
        //   }
        //   case RIDE:{//going to a single locatio
        //     break;
        //   }
        //   case PROD:{//investigating obstacle
        //     break;
        //   }
        //   case DONE:{//exploration complete
        //     done = true;
        //     break;
        //   }
        // }


      ros::Duration(0.5).sleep();
      if(done) break;
      else{
        ros::spinOnce();
        step++;
      }
    }

  }
}
