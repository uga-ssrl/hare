# HARE
[Heterogeneous Autonomous Multirobot Exploration Ros Node](https://github.com/uga-ssrl/hare)


You can read the in-progress [IEEE style paper](https://github.com/uga-ssrl/hare/blob/master/doc/hare-paper.pdf) at: https://github.com/uga-ssrl/hare/blob/master/doc/hare-paper.pdf, this is also located in this repo in the `doc` folder.

--------
# Dependencies
**ros-behavior-tree**
- git clone https://github.com/miccol/ROS-Behavior-Tree.git in ~/catkin_ws/src/

**Enviroment Variables**
- The `GAZEBO_MODEL_PATH` needs to point to the `world` folder in the hare repository. This should be a global / absolute path. **OR!** simply go to the world folder in this repo and type: `export GAZEBO_MODEL_PATH=$(catkin_ws)/src/hare/world`

**Turtlebot-Navigation**
- Just need to install `sudo apt-get install ros-kinetic-turtlebot-navigation`

# Robot Dependencies
- clone all of the following in ```~/catkin_ws/src/robot/```

**robot1**
- turtlebot_description package for launch files

**(youbot variant)**
- in /path/to/catkin_ws/src ```git clone https://github.com/youbot/youbot_simulation.git```
- in /path/to/catkin_ws/src ```git clone https://github.com/youbot/youbot_description.git --branch kinetic-devel```


**robot2**
- update update rate to 50 at line 5 of rosbot.gazebo
- in /path/to/catkin_ws/src/ ```git clone  https://github.com/husarion/rosbot_description.git```
- in /path/to/rosbot_description/urdf/rosbot.gazebo find the following commented line and follow the instructions to not use GPU ```  <!-- If you cant't use your GPU comment RpLidar using GPU and uncomment RpLidar using CPU
    gazebo plugin. -->```


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
- you must also increment the following in robots.launch ```<param name="num_robots" value="3"/>```

#ERROR <LegaceModeNS> mitigation for gazebo plugin
- go into your ```.gazebo``` in each robot description, i.e: for youbot go into: ```youbot_description/controller/ros_controller.urdf.xacro``` and add the following snippet:

```
<gazebo>
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
- Odom in global frame to use obstacle sensing
- work on steering mode
- more reliable controller for heterogeneous types
**nodes to be implemented**
- explore_multirobot
- tree node
