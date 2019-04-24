# HARE
[Heterogeneous Autonomous Multirobot Exploration Ros Node](https://github.com/uga-ssrl/hare)

--------
# Dependencies
**ros-behavior-tree**
- git clone https://github.com/miccol/ROS-Behavior-Tree.git in ~/catkin_ws/src/

**multi-robo-map-merge**
- multirobot_map_merge (https://wiki.ros.org/multirobot_map_merge)
- to install run: ```sudo apt install ros-kinetic-multirobot-map-merge ros-kinetic-explore-lite```

**Enviroment Variables**
- The `GAZEBO_MODEL_PATH` needs to point to the `world` folder in the hare repository. This should be a global / absolute path. **OR!** simply go to the world folder in this repo and type: `export GAZEBO_MODEL_PATH=$(pwd)`

**Turtlebot-Navigation**
- Just need to install `sudo apt-get install ros-kinetic-turtlebot-navigation`

**robot1**
- turtlebot_description package for launch files


**(youbot variant)**
-[Install](http://www.youbot-store.com/wiki/index.php/Gazebo_simulation) KUKA model robot, and robot drivers.

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

#ERROR <LegaceModeNS> mitigation for gazebo plugin
- go into your ```.gazebo``` in each robot description, i.e: for youbot go into: ```youbot_description/controller/ros_controller.urdf.xacro``` and add the following snippet:

```<gazebo>
<static>false</static>

<plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
  <robotNamespace>\robotNamespace</robotNamespace>
  <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
  <legacyModeNS>true</legacyModeNS>
</plugin>
</gazebo>
```
- Where robotNamespace is the robot you are talking about

# TODOs
**nodes to be added**
- multi_robot_collision_avoidance (https://wiki.ros.org/multi_robot_collision_avoidance)
- explore_multirobot?
- tuw_multi_robot?
- MULTIMASTER (POUND - see doc/pound.pdf)
- rosbot localization
- Odom in global frame to use obstacle sensing
- path planning (meander or command based)

**nodes to be implemented**
- hare (actual exploration part)
