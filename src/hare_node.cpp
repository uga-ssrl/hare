#include "common_includes.h"
#include "Entity.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "hare_node");
  ros::NodeHandle nh;
  hare::Robot robot = hare::Robot(nh);
  robot.setQueueSize(1000);
  ros::Duration(3.0).sleep();//wait for gazebo to start
  robot.run();
  return 0;
}
