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
  this->velocity = 0.0;
}
hare::Neighbor::~Neighbor(){

}

hare::Robot::Robot(){
  this->id = -1;
  this->ns = "/";
  this->type = ROBOT;
  this->linear.x = 0.0;
  this->linear.y = 0.0;
}
//set queue size in here
hare::Robot::Robot(ros::NodeHandle nh){
  this->nh = nh;
  this->ns = nh.getNamespace();
  this->id = std::stoi(this->ns.substr(this->ns.length() - 1, 1));
  this->type = ROBOT;
  this->nh.getParam("init_x", this->pos.x);
  this->nh.getParam("init_y", this->pos.y);
  this->nh.getParam("init_z", this->pos.z);
}
hare::Robot::~Robot(){

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
  ros::Publisher test_pub = this->nh.advertise<std_msgs::String>("test_msg", this->queue_size);
  this->addPublisher(test_pub);
  std::string ns = this->ns;
  // ros::Publisher cmd_vel = this->nh.advertise<geometry_msgs::Twist>("cmd_vel", this->queue_size);
  // this->addPublisher(cmd_vel);
  // change me
  ros::Publisher twist_pub = this->nh.advertise<geometry_msgs::Twist>("goal", this->queue_size);
  this->addPublisher(twist_pub);

}
void hare::Robot::initSubscribers(){
  //single point subscriptions


  //per neighbor subscriptions
  for(auto neighbor = this->neighbors.begin(); neighbor != this->neighbors.end(); ++neighbor){
    std::string neighbor_ns = (*neighbor).ns;
    ros::Subscriber test_sub = this->nh.subscribe<std_msgs::String>(neighbor_ns + "/test_msg", this->queue_size, &hare::Robot::callback, this);
    this->addSubscriber(test_sub);
    ros::Subscriber odom = this->nh.subscribe<nav_msgs::Odometry>(neighbor_ns + "/odom", this->queue_size, &hare::Robot::callback, this);
    this->addSubscriber(odom);
    ros::Subscriber nav_subs = this->nh.subscribe<hare::HareUpdate>(neighbor_ns + "/HARE", this->queue_size, &hare::Robot::callback, this);
    this->addSubscriber(nav_subs);

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

void hare::Robot::callback(const hare::HareUpdateConstPtr& msg){
  //this->robot_id = msg->robot_id
  //this->walls = msg->walls
  //this->odom = msg->robot_odom
  //this->tree_state = msg->tree_state

}

void hare::Robot::callback(const nav_msgs::OdometryConstPtr& msg){


}

void hare::Robot::setCallBackQueue(ros::CallbackQueue callbackQueue){
  this->nh.setCallbackQueue(&callbackQueue);
}

std::vector<map_node> hare::Robot::sense(float range){
  std::vector<int2> cells;
  int2 minBound = {floor(this->pos.x-range),floor(this->pos.y-range)};
  int2 maxBound = {floor(this->pos.x+range),floor(this->pos.y+range)};

}

void hare::Robot::run(){
  std_msgs::String str;
  hare::HareUpdate neighbor_states;
  hare::HareUpdate my_state;
  // float2 goal;
  // float2 start;

  geometry_msgs::Twist goal = geometry_msgs::Twist();
  geometry_msgs::Twist start = geometry_msgs::Twist();

  // start.x = 0;
  // start.y = 0;
  //
  // goal.x = 0;
  // goal.y = 0;


  std::vector<float2>trajectory;

  //this->my_state = my_state;
  //this->neighbor_states = neighbor_states;

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
    //this->map = sense();
    for(auto neighbor = this->neighbors.begin(); neighbor != this->neighbors.end(); ++neighbor)
    {
          std::string neighbor_ns = (*neighbor).ns;
          // ros::Subscriber sub = this->nh.subscribe<hare::HareUpdate>(neighbor_ns + "", this->queue_size, &hare::Robot::callback, this);
    }
    //TODO
    // this subscriber will update the class
    //

    //PERFORM ACTION BASED ON GOAL AND STATE
    geometry_msgs::Twist msg;
    // hare::HareUpdate information_state;
    switch(this->tree_state)
    {
      case 1:
        //DFS();
        break;
      case 2:
        //msg = getPath(this->description, start, goal);
        break;
      case 3:
        //DFS();
        break;
      default:
        break;
    }


    //SEND INFORMATION TO NEIGHBORS
    this->publish<geometry_msgs::Twist>(goal,"goal");
    // this->publish<geometry_msgs::Twist>(msg,"/cmd_vel");
    // this->publish<hare::HareUpdate>(information_state,"/HARE");
    ros::spinOnce();
  }
}
