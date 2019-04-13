#ifndef ENTITY_H
#define ENTITY_H

#include "common_includes.h"
#include "utility.h"
#include <ros/callback_queue.h>

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
    float3 pos;
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
    float velocity;
    float3 orientation;

    Neighbor();
    Neighbor(std::string ns);
    ~Neighbor();

  };

  class Robot : public Entity{

    ros::NodeHandle nh;
    std::map<std::string, int> publisherMap;
    std::vector<ros::Publisher> publishers;
    std::vector<ros::Subscriber> subscribers;
    void loadCapabilties();
    void findNeighbors();
    bool addPublisher(ros::Publisher &pub);
    bool addSubscriber(ros::Subscriber &sub);

  public:

    RobotDescription description;
    float velocity;
    float3 orientation;

    std::vector<Neighbor> neighbors;

    Robot();
    Robot(ros::NodeHandle nh);
    ~Robot();

    void initComms(uint32_t queue_size);

    //TODO implement all callbacks
    void callback(const std_msgs::StringConstPtr& str);
    void setCallBackQueue(ros::CallbackQueue callbackQueue);

    void run();

    template <typename T>
    void publish(T message, std::string topic);

  };
  template <typename T>
  void Robot::publish(T message, std::string topic){
    if(this->publisherMap.find(topic) == this->publisherMap.end()){
      ROS_ERROR("There is no publisher for the given topic %s", topic.c_str());
    }
    this->publishers[this->publisherMap[topic]].publish(message);
  }
}


#endif /* ENTITY_H */
