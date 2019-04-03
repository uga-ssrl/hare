#include <climits>
#include <stdlib.h>
#include <time.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <vector>
#include "common_includes.h"
#include <string>
#include <iostream>

void callback(const std_msgs::StringConstPtr& str){
  std::cout<<str->data<<std::endl;
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
      neighbor_ns[i] = "/" + neighbor_ns[i];
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
  }

  std_msgs::String str;
  str.data = "my second friend is " + neighbor_ns[numNeighbors-1];

  ros::Rate r(10);//10 hz

  while (ros::ok()){
    pub.publish(str);
		ros::spinOnce();
		r.sleep();
  }

  return 0;
}
