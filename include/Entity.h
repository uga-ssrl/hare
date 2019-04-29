#ifndef ENTITY_H
#define ENTITY_H

#include "common_includes.h"
#include "utility.h"
#include <ros/callback_queue.h>
#include <hare/cell.h>
#include <hare/HareUpdate.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include "Map.h"

namespace hare{

  typedef struct EntityDescription{
    float3 boundingBox;
  } EntityDescription;

  typedef struct RobotDescription : public EntityDescription{
    bool canFly;
    bool waterResistance;//0 = not waterproof, 1 = water resistant, 2 = water proof
    float torque;
    float turnRadius;
    float weight;
  } RobotDescription;

  class Entity{

  public:
    EntityDescription description;

    nav_msgs::Odometry odom;

    std::string state_indicator;
    std::string ns;
    int id;
    EntityType type;
    Entity();
    Entity(std::string ns);
    ~Entity();

  };

  class Neighbor : public Entity{

  public:
    RobotDescription description;

    Neighbor();
    Neighbor(std::string ns);
    ~Neighbor();

  };

  class Robot : public Entity{

    uint32_t queue_size;
    Map* map;
    std::vector<float3> path;


    ros::NodeHandle nh;
    std::map<std::string, int> publisherMap;
    std::vector<ros::Publisher> publishers;
    std::vector<ros::Subscriber> subscribers;
    void loadCapabilties();
    void findNeighbors();
    bool addPublisher(ros::Publisher &pub);
    bool addSubscriber(ros::Subscriber &sub);

    void initPublishers();
    void initSubscribers();

    //minMax is in form of fullMap indices
    //NOTE the sensed region is from the pos of robot odom
    void sense(std::vector<hare::map_node>& region, int4 &minMax);

    //HARE OPERABLE METHODS
    //TODO implement and test
    void investigateObject();//wall follow
    void findCapableNeighbor();//determine who can maneuver
    void notifyNeighbor();//tell neighbor that there is an obstacle it should visit
    void switchWithNeighbor(); //switch locations with neighbor
    void search();//heuristic search

    bool isDone();//no more unexplored cells


  public:

    RobotDescription description;
    int tree_state;
    int4 linear;
    std::vector<Neighbor> neighbors;

    Robot();
    //NOTE INITIAL POSITIONS SET AS PARAMETERS IN LAUNCH FILES
    //ARE TRANSLATED IN THE FOLLOWING CONSTRUCTOR
    Robot(ros::NodeHandle nh);
    ~Robot();

    void setQueueSize(uint32_t queue_size);
    void init();

    //TODO implement all callbacks
    void callback(const std_msgs::StringConstPtr& msg);
    void callback(const hare::HareUpdateConstPtr& msg);
    void callback(const nav_msgs::OdometryConstPtr& msg);
    void setCallBackQueue(ros::CallbackQueue callbackQueue);

    //NECESSITATES THAT ROBOT
    //NOTE NEIGHBORING ROBOTS INITIAL POSITION IS SET IN FIRST LOOP
    void run();

    template <typename T>
    void publish(T message, std::string topic);
  };
  template <typename T>
  void Robot::publish(T message, std::string topic){
    topic = this->ns + "/" + topic;
    if(this->publisherMap.find(topic) == this->publisherMap.end()){
      ROS_ERROR("There is no publisher for the given topic %s", topic.c_str());
    }
    this->publishers[this->publisherMap[topic]].publish(message);
  }
}


#endif /* ENTITY_H */
