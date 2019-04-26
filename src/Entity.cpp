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
}
//set queue size in here
hare::Robot::Robot(ros::NodeHandle nh){
  this->nh = nh;
  this->ns = nh.getNamespace();
  this->id = std::stoi(this->ns.substr(this->ns.length() - 1, 1));
  this->type = ROBOT;
}
hare::Robot::~Robot(){

}
//must add capabilities here if you add them to yaml file
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
void hare::Robot::initPublishers(){
  ros::Publisher test_pub = this->nh.advertise<std_msgs::String>("test_msg", this->queue_size);
  std::cout<<test_pub.getTopic()<<std::endl;
  this->addPublisher(test_pub);
}
void hare::Robot::initSubscribers(){
  ros::Subscriber obst_sub = this->nh.subscribe<hare::Obstacle>("obstacle_sensing", this->queue_size, &hare::Robot::callback, this);
  this->addSubscriber(obst_sub);

  for(auto neighbor = this->neighbors.begin(); neighbor != this->neighbors.end(); ++neighbor){
    std::string topic = (*neighbor).ns + "/test_msg";
    ros::Subscriber test_sub = this->nh.subscribe<std_msgs::String>(topic, this->queue_size, &hare::Robot::callback, this);
    this->addSubscriber(test_sub);
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
  //ROS_INFO("received %s", msg->data.c_str());
}
void hare::Robot::callback(const hare::ObstacleConstPtr& msg){

}
void hare::Robot::callback(const hare::HareUpdateConstPtr& msg){

}
void hare::Robot::setCallBackQueue(ros::CallbackQueue callbackQueue){
  this->nh.setCallbackQueue(&callbackQueue);
}

void hare::Robot::run(){
  std_msgs::String str;
  str.data = "hi from " + this->id;

  //set spinner here if using anything but SingleSpinner
  std::string test_topic;
  if(nh.getParam("/sub_and_pub/test", test_topic)){
    ROS_INFO("Got param for publishing: %s", test_topic.c_str());
  }
  else{
    ROS_ERROR("Failed to get param 'test'");
  }
  while (ros::ok()){

    //DETERMINE STATE AND SET GOALS
      //neighbor state query
      //mapping
      //obstacle identification
      //obstacle avoidance
      //trajectory
      //path planning

    //PERFORM ACTION BASED ON GOAL AND STATE
      //behavior tree traversal

    //SEND INFORMATION TO NEIGHBORS
    this->publish<std_msgs::String>(str,test_topic);
    ros::spinOnce();
  }
}
