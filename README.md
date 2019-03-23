# HARE
Heterogeneous Autonomous Multirobot Exploration Ros Node

# Dependencies
**ros-behavior-tree**
- git clone https://github.com/miccol/ROS-Behavior-Tree.git in ~/catkin_ws/src/

**robot1**
- turtlebot_description package for launch files


**robot2**
- clone [this](https://github.com/Spain2394/MMP30_Robot.git) repo and add the following packages to your ```~/catkin_ws/src/robot/```
- mmp30_gazebo
- mmp30_control
- mmp30_description


**robot3**
- clone https://github.com/husarion/rosbot_description in ```~/catkin_ws/src/robot/```


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
