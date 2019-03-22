# HARE
Heterogeneous Autonomous Multirobot Exploration Ros Node

# Dependencies
- follow install instructions for buzz https://github.com/MISTLab/Buzz.git
- clone rosbuzz https://github.com/MISTLab/ROSBuzz.git within src of catkin_ws
  - need to change bzzfile_name param in rosbuzz.launch
`<param name="bzzfile_name" value="$(find rosbuzz)/buzz_scripts/$(arg script).bzz"/>`
to
`<param name="bzzfile_name" value="$(arg script)"/>`

**robot1**
- turtlebot_description package for launch files
**robot3**
- clone [this](https://github.com/Spain2394/MMP30_Robot.git) repo and add the following packages to your ```~/catkin_ws/src```
- mmp30_gazebo
- mmp30_control
- mmp30_description

**robot**
- clone https://github.com/husarion/rosbot_description within src of catkin_ws
- gazebo


# Compilation
`/path/to/catkin_ws$ catkin_make -DKIN=1 -DSIM=1`
then
`/path/to/catkin_ws$ source devel/setup.bash`

# Simulation
`roslaunch hare hare_sim.launch`

# Buzz development
- To change the buzz script that is being run on the robots just change script argument in
hare/launch/one_robot.launch
- Buzz scripts should be written in hare/buzz_scripts

# Adding robots
- add robots in hare/launch/robots.launch
- create new namespace and place descriptions and include one_robot.launch
  - this can be seen in the turtlebot examples
