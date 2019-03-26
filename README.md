# HARE
[Heterogeneous Autonomous Multirobot Exploration Ros Node](https://github.com/uga-ssrl/hare)

--------
# Dependencies
**ros-behavior-tree**
- git clone https://github.com/miccol/ROS-Behavior-Tree.git in ~/catkin_ws/src/

**multi-robo-map-merge**
- multirobot_map_merge (https://wiki.ros.org/multirobot_map_merge)
- to install run: sudo apt install ros-kinetic-multirobot-map-merge ros-kinetic-explore-lite


**robot1**
- turtlebot_description package for launch files

**robot2**
- clone https://github.com/husarion/rosbot_description in ```~/catkin_ws/src/robot/```

**robot3**
- To install husky packages run: ```sudo apt-get install ros-kinetic-husky-simulator```

# Compilation
`/path/to/catkin_ws$ catkin_make`
then
`/path/to/catkin_ws$ source devel/setup.bash`

# Simulation
`roslaunch hare hare_sim.launch`

# Adding robots
- add robots in hare/launch/robots.launch
- create new namespace and place descriptions and include one_robot.launch
  - this can be seen in the turtlebot examples

# TODOs
**nodes to be added**
- multi_robot_collision_avoidance (https://wiki.ros.org/multi_robot_collision_avoidance)
- explore_multirobot?
- tuw_multi_robot?

**nodes to be implemented**
- capabilities
- hare_msgs
- hare (actual exploration part)
