# HARE
Heterogeneous Autonomous Multirobot Exploration System

# Dependencies
- buzz https://github.com/MISTLab/Buzz.git
- rosbuzz https://github.com/MISTLab/ROSBuzz.git
- gazebo
- turtlebot_description package for launch files
- need to change bzzfile_name param in rosbuzz.launch
`<param name="bzzfile_name" value="$(find rosbuzz)/buzz_scripts/$(arg script).bzz"/>`
to
`<param name="bzzfile_name" value="$(arg script)"/>`

 - this is necessary as hare_sim.launch sets script arg to hare.bzz

# Compilation
`catkin_make -DKIN=1 -DSIM=1`
