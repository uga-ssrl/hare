/* RosBuzz.cpp -- Basic ROS Buzz node --               */
/* ------------------------------------------------------------------------- */
/* September 20, 2016 -- @Copyright Aymen Soussia. All rights reserved.      */
/*                                  (aymen.soussia@gmail.com)                */


#include <climits>
#include <stdlib.h>
#include <time.h>
#include <ros/ros.h>
#include "common_includes.h"

void callback(){

}

int main(int argc, char **argv){
  ros::init(argc, argv, "test_buzz");

  ros::NodeHandle nh;

	// robot1_in = nh1.advertise<mavros_msgs::Mavlink>("/robot1/inMavlink", 100);
  // robot2_in = nh2.advertise<mavros_msgs::Mavlink>("/robot2/inMavlink", 100);
	// robot1_out = nh2.subscribe("/robot1/outMavlink", 100, callback1);
	// robot2_out = nh1.subscribe("/robot2/outMavlink", 100, callback2);

  ros::Rate r(10);//10 hz

  while (ros::ok()){
		ros::spinOnce();
		r.sleep();
  }

  return 0;
}
