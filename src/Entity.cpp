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
hare::Robot::Robot(ros::NodeHandle nh){
  this->nh = nh;
  this->ns = nh.getNamespace();
  this->id = std::stoi(this->ns.substr(this->ns.length() - 1, 1));
  this->type = ROBOT;
  this->findNeighbors();
  this->initComms(500);
  this->run();
}
hare::Robot::~Robot(){

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
void hare::Robot::initComms(uint32_t queue_size){
  //add pubs and subs
  std::string test_topic;
  if(this->nh.getParam("/robots_sub_pub/test", test_topic)){
    ROS_INFO("Got param: %s", test_topic.c_str());
  }
  else{
    ROS_ERROR("Failed to get param 'test'");
  }

  ros::Publisher pub = this->nh.advertise<std_msgs::String>(test_topic, queue_size);
  if(!pub){
    ROS_ERROR("failed attempt to add bad publisher");
  }
  else{
    this->publishers.push_back(pub);
    this->publisherMap.insert(std::make_pair(test_topic, this->publishers.size() - 1));
  }

  for(auto neighbor = this->neighbors.begin(); neighbor != this->neighbors.end(); ++neighbor){
    std::string topic = (*neighbor).ns + "/" + test_topic;
    ros::Subscriber sub = this->nh.subscribe<std_msgs::String>(topic, queue_size, &hare::Robot::callback, this);
    if(!sub){
      ROS_ERROR("failed attempt to add bad subscriber");
      return;
    }
    else{
      this->subscribers.push_back(sub);
    }
  }
}

void hare::Robot::callback(const std_msgs::StringConstPtr& str){
  ROS_INFO("received %s", str->data);
}
void hare::Robot::setCallBackQueue(ros::CallbackQueue callbackQueue){
  this->nh.setCallbackQueue(&callbackQueue);
}

void hare::Robot::run(){
  std_msgs::String str;
  str.data = "hi from " + this->id;

  //set spinner here if using anything but SingleSpinner
  std::string test_topic;
  if(nh.getParam("/robots_sub_pub/test", test_topic)){
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
