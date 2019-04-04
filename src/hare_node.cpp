#include <climits>
#include <stdlib.h>
#include <time.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <vector>
#include "common_includes.h"
#include <string>
#include <iostream>

// class CommunicationHandler{
//   std::vector<ros::Publisher> publishers;
//   std::vector<ros::Subscriber> subscribers;
//
// public:
//
//   CommunicationHandler();
//
// };
//
// struct Neighbor{
//   int id;
//   std::string ns;
//
// };
//
// class Robot{
//
//   CommunicationHandler* communicator;
//
// public:
//
//   std::vector<Neighbor> neighbors;
//
//   Robot();
//
// };

void callback(const std_msgs::StringConstPtr& str){
  ROS_INFO("received %s", str->data);
 }

int main(int argc, char **argv){
  ros::init(argc, argv, "hare_node");
  ros::NodeHandle nh;

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
  std::vector<std::string> neighbor_ns;
  for(int i = 0; i < numNeighbors; ++i){
    temp = "neighbor" + std::to_string(i + 1);
    std::string placeHolder;
    neighbor_ns.push_back(placeHolder);
    if(nh.getParam(temp,neighbor_ns[i])){
      neighbor_ns[i] =  "/" + neighbor_ns[i];
      ROS_INFO("Got param: %s", neighbor_ns[i].c_str());
    }
    else{
      ROS_ERROR("Failed to get param 'neighbor%d'", i + 1);
    }
  }

  ros::Publisher pub = nh.advertise<std_msgs::String>("test_msg", 1);

  std::vector<ros::Subscriber> subs;
  for(int i = 0; i < numNeighbors; ++i){
    temp = neighbor_ns[i] + "/test_msg";
    ros::Subscriber sub = nh.subscribe(temp, 1, callback);
    subs.push_back(sub);
  }

  std_msgs::String str;
  str.data = "my second friend is " + neighbor_ns[numNeighbors-1];

  ros::Rate r(100);//100 hz

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
    pub.publish(str);

		ros::spinOnce();
  }

  return 0;
}
