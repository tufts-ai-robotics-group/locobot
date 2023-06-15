# Locobot_custom
Custom ROS package to run planning/learning experiments with Locobot robot in simulation. Built for ROS Noetic.

#### Launch the simulation with the pre-built map

```
roslaunch locobot_custom nav_moveit.launch
```
Note: The RTABmap is located in ~/.ros/rtabmap.db

#### Run the script for sending the goal to go to locations

```
rosrun locobot_custom move_base_loco.py <objects>
```
objects: table, doorway_facing_green, doorway_facing_blue, large_sofa, small_sofa, bin

#### Script to obtain the Pose of the 2D nav goals issues through Rviz

```
rosrun locobot_custom obtain_nav_goals.py
```