#ifndef ENTITY_H
#define ENTITY_H

#include "common_includes.h"
#include <ros/callback_queue.h>

namespace hare{
  class Entity{

  public:
    std::string ns;
    int id;
    EntityType type;
    Entity();
    Entity(std::string ns);
    ~Entity();

  };

  class Neighbor : public Entity{

  public:
    Neighbor();
    Neighbor(std::string ns);
    ~Neighbor();

  };

  class Robot : public Entity{

    ros::NodeHandle nh;
    std::map<std::string, int> publisherMap;
    std::vector<ros::Publisher> publishers;
    std::vector<ros::Subscriber> subscribers;
    void findNeighbors();
    void initComms(uint32_t queue_size);

  public:

    std::vector<Neighbor> neighbors;

    Robot();
    Robot(ros::NodeHandle nh);
    ~Robot();

    //TODO implement all callbacks
    void callback(const std_msgs::StringConstPtr& str);
    void setCallBackQueue(ros::CallbackQueue callbackQueue);

    void run();

    template <typename T>
    void publish(T message, std::string topic);

  };
  template <typename T>
  void Robot::publish(T message, std::string topic){
    std::cout<<"attempting to publish "<<topic<<std::endl;
    if(this->publisherMap.find(topic) == this->publisherMap.end()){
      ROS_ERROR("There is no publisher for the given topic %s", topic);
    }
    else{
      ROS_INFO("Publishing to topic %s",topic.c_str());
      std::cout<<"found publisher"<<std::endl;
    }
    this->publishers[this->publisherMap[topic]].publish(message);
    std::cout<<"successfully posted"<<std::endl;
  }
}


#endif /* ENTITY_H */
