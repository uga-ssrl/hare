#include "Entity.h"

hare::Entity::Entity(){
  this->id = -1;
  this->ns = "/";
  this->type = UNKNOWN;
}
hare::Entity::Entity(std::string ns){
  this->ns = ns;
  this->id = std::stoi(this->ns.substr(this->ns.length() - 1, 1));
  this->type = UNKNOWN;
}
hare::Entity::~Entity(){

}

hare::Neighbor::Neighbor(){
  this->id = -1;
  this->ns = "/";
  this->type = ROBOT;
}
hare::Neighbor::Neighbor(std::string ns){
  this->ns = ns;
  this->id = std::stoi(this->ns.substr(this->ns.length() - 1, 1));
  this->type = ROBOT;
}
hare::Neighbor::~Neighbor(){

}

hare::Robot::Robot(){
  this->id = -1;
  this->ns = "/";
  this->type = ROBOT;
  this->linear.x = 0.0;
  this->linear.y = 0.0;
  this->map = NULL;
}
//set queue size in here
hare::Robot::Robot(ros::NodeHandle nh){
  this->nh = nh;
  this->ns = nh.getNamespace();
  this->id = std::stoi(this->ns.substr(this->ns.length() - 1, 1));
  this->type = ROBOT;
  float3 position;
  this->nh.getParam("init_x", position.x);
  this->nh.getParam("init_y", position.y);
  this->nh.getParam("init_z", position.z);
  this->path.push_back({(position.x*ODOM_TO_MAP)+(MAP_X/2),
    (position.y*ODOM_TO_MAP)+(MAP_Y/2),
    (position.z*ODOM_TO_MAP)});
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
  if(!this->nh.getParam("/static_characteristics" + this->ns + "/turnRadius", this->description.turnRadius)){
    ROS_ERROR("Failed to get param turnRadius");
    received = false;
  }
  if(!this->nh.getParam("/static_characteristics" + this->ns + "/weight", this->description.weight)){
    ROS_ERROR("Failed to get param weight");
    received = false;
  }
  if(!this->nh.getParam("/static_characteristics" + this->ns + "/torque", this->description.torque)){
    ROS_ERROR("Failed to get param torque");
    received = false;
  }
  if(!this->nh.getParam("/static_characteristics" + this->ns + "/canFly", this->description.canFly)){
    ROS_ERROR("Failed to get param canFly");
    received = false;
  }
  if(received){
    ROS_INFO("Successfuly loaded capabilities from yaml file");
  }
  std::vector<float> temp;
  if(!this->nh.getParam("/static_characteristics" + this->ns + "/boundingBox", temp)){
    ROS_ERROR("Failed to get param boundingBox");
  }
  this->description.boundingBox.x = temp[0];
  this->description.boundingBox.y = temp[1];
  this->description.boundingBox.z = temp[2];

  for(auto neighbor = this->neighbors.begin(); neighbor != this->neighbors.end(); ++neighbor){
    received = true;
    std::vector<float> neighborTemp;
    if(!this->nh.getParam("/static_characteristics" + (*neighbor).ns + "/turnRadius", (*neighbor).description.turnRadius)){
      ROS_ERROR("Failed to get param turnRadius");
      received = false;
    }
    if(!this->nh.getParam("/static_characteristics" + (*neighbor).ns + "/weight", (*neighbor).description.weight)){
      ROS_ERROR("Failed to get param weight");
      received = false;
    }
    if(!this->nh.getParam("/static_characteristics" + (*neighbor).ns + "/torque", (*neighbor).description.torque)){
      ROS_ERROR("Failed to get param torque");
      received = false;
    }
    if(!this->nh.getParam("/static_characteristics" + (*neighbor).ns + "/canFly", (*neighbor).description.canFly)){
      ROS_ERROR("Failed to get param canFly");
      received = false;
    }
    if(!this->nh.getParam("/static_characteristics" + this->ns + "/boundingBox", neighborTemp)){
      ROS_ERROR("Failed to get param boundingBox");
      received = false;
    }
    (*neighbor).description.boundingBox.x = neighborTemp[0];
    (*neighbor).description.boundingBox.y = neighborTemp[1];
    (*neighbor).description.boundingBox.z = neighborTemp[2];
    if(received){
      ROS_INFO("Successfully loaded capabilities of %s", (*neighbor).ns.c_str());
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
      (*neighbor).state_indicator = msg->tree_state.data;
      break;
    }
  }
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

void hare::Robot::run(){
  hare::HareUpdate update;
  update.robot_id = this->id;
  update.tree_state.data = "starting";

  std::vector<hare::map_node> sensedRegion;
  int4 sensoryBound = {0,0,0,0};//{min.x,min.y,max.x,max.y} - indices in fullMap
  float sensingRange = 1.0f; unsigned int step = 0; float3 currentPosition;

  while (ros::ok()){
    //THIS NEEDS TO BE TRANSLATED FROM 0,0 to 200,200
    currentPosition = {(this->odom.pose.pose.position.x*ODOM_TO_MAP)+MAP_X/2,
      (this->odom.pose.pose.position.y*ODOM_TO_MAP)+MAP_Y/2,
      (this->odom.pose.pose.position.z*ODOM_TO_MAP)};
    //MAKE SURE TO TRANSLATE THIS POSITION TO OUR COORDINATE FRAME
    if(step != 0 &&
    this->path[this->path.size()-1].x != currentPosition.x &&
    this->path[this->path.size()-1].y != currentPosition.y){
      update.odom = this->odom;
      sensedRegion.clear();
      sensoryBound = {floor(currentPosition.x-sensingRange),
        floor(currentPosition.y-sensingRange),
        floor(currentPosition.x+sensingRange),
        floor(currentPosition.y+sensingRange)
      };
      this->sense(sensedRegion,sensoryBound);
      this->map->update(sensoryBound,sensedRegion);
      this->path.push_back(currentPosition);
    }


    update.cells.clear();
    for(int x = sensoryBound.x, i = 0; x < sensoryBound.z; ++x){
      for(int y = sensoryBound.y; y < sensoryBound.w; ++y){
        hare::cell _cell;
        _cell.x = x;
        _cell.y = y;
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
    this->publish<hare::HareUpdate>(update,"HARE_UPDATE");

    //TREE STUFF

    ros::spinOnce();
    step++;
  }
}
