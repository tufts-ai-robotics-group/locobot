# Locobot_custom
Custom ROS package to run planning/learning experiments with Locobot robot in simulation. Built for ROS Noetic.

## Simulation
#### Launch the simulation with the pre-built map

```
roslaunch locobot_custom nav_moveit.launch
```
Note: The RTABmap is located in `~/.ros/rtabmap`.

## Real Robot
### Robot NUC Computer
If you want to build a new map:
```
roslaunch locobot_custom nav_moveit.launch localization:=false rtabmap_args:=-d
```

Or if you want to use a stored map and run localization:
```
roslaunch locobot_custom nav_moveit.launch localization:=true
```
Note: The RTABmap is located in `~/.ros/rtabmap` on the on-board NUC computer.

### Remote Control on another computer
Firstly, make sure ROS_IP and ROS_MASTER_URI is correctly set:
```
export ROS_IP=<your desktop computer IP>
export ROS_MASTER_URI=http://locobot.local:11311
```

Secondly, launch RViz by running the following command:
```
roslaunch locobot_custom remote_view.launch
```

## Navigation Using Python Code (works in both simulation and real robot)
#### Run the script for sending the goal to go to locations

```
rosrun locobot_custom move_base_loco.py <objects>
```
objects: table, doorway_facing_green, doorway_facing_blue, large_sofa, small_sofa, bin

#### Script to obtain the Pose of the 2D nav goals issued through Rviz

```
rosrun locobot_custom obtain_nav_goals.py
```


## TroubleShooting
### Time synchronization with the Create 3 Base
If you see this issue:
```
Lookup would require extrapolation ... into the future.
```
Then this is an issue of the main NUC computer and the Create 3 base (it has its own computer) out of sync. 
It might happen when the robot has not been on for a while and is just booted up. 
Normally it should sync itself after a few seconds. 
If time synchronization fail to happen, consult the troubleshoot page [here](https://docs.trossenrobotics.com/interbotix_xslocobots_docs/troubleshooting.html#create-3-base-clock-is-not-synchronized).

### Arm Failure
If the arm collided with some item while moving and red indicator light of one or more of the joints are flashing,
first hold the arm and turn off the ros node. Put it back to its sleep position, then
unplug the 12V connection from the large powerbank and plug it in again after waiting 5 seconds. It should work
again after this reset.
