# locobot_custom
Custom ROS package to run Planning/Learning Experiments with Locobot Robot in Simulation and Real World.
Built for [ROS Noetic](http://wiki.ros.org/noetic).
Tested in Ubuntu 20.04 & Python = 3.8.10

File Structure
```
.
├── config
├── launch
├── rviz
├── scripts
│   ├── environment            # MDP 
│   │   ├── __init__.py
│   │   ├── RecycleBot.py                       # Environment Class
│   │   ├── ActionGenerator.py                  # Action Generator
│   │   ├── ObservationGenerator.py             # Observation Generator
│   │   ├── RewardFunction.py                   # Reward function Generator
│   ├── executor                # Executors
│   │ ├── __init__.py
│   │   ├── approach.py
│   │   ├── grasp.py
│   │   ├── place.py
│   │   └── universal_move_arm.py
│   ├── knowledge                  
│   │   ├── PDDL                                 # PDDL files
│   │   └── pddl-parser                          # PDDL parsing
│   ├── agent                   # Agent
│   │   ├── __init__.py
│   │   ├── RecycleBotPlanner.py                # Planning agent
│   │   ├── dummy_planner.py
│   │   ├── learner                             # Reinforcement Learning agent
│   │   └── planner
│   ├── manager                 # Run
│   │   ├── __init__.py
│   │   ├── experiment.py
├── srv
└── worlds
```


## Simulation
#### Launch the simulation with the pre-built map

```
roslaunch locobot_custom nav_moveit.launch
```
Note: The RTABmap is located in `~/.ros/rtabmap`.

#### Obtain the location of an object in Gazebo
Run
```
rosrun locobot_custom obtain_obj_loc.py <obj_name>
```
to obtain and print the location of the objects. If no object name is specified, the list of available objects' names will be printed.

#### Move the arm to a specific location

The main code component is the `LBMoveIt` class, which automatically
registers the move group and allows you to move it to a joint
state or a pose goal. For more example, please see the official
example code from
`[interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_moveit_interface/scripts/moveit_python_interface]`. [Link](https://github.com/Interbotix/interbotix_ros_toolboxes/blob/main/interbotix_common_toolbox/interbotix_moveit_interface/scripts/moveit_python_interface)

To directly run the test code and script, do:
```
ROS_NAMESPACE=locobot rosrun locobot_custom universal_move_arm.py <args>
```
The script also includes three different test functions. If you run the script directly, you may specify `<args>` depending on which test function you want to run:
- `main` function tries to get the pickup pose using the code in `obtain_pick_item_pose.py` and moves the arm to the item. Run the script without args to run this function.
- `test1` function tries to move the arm to a specific location, which can be helpful for debugging. Run the script with argument "test" to run this function.
- `print_pose` function prints the current pose of the arm, which could also be helpful for debugging when for example we manually set the arm to a specific angle and we want to see what pose the arm is at. Run the script with argument get_pose to run this function.

#### Obtain the calculated pickup pose given the object name in Gazebo
Run
```
rosrun locobot_custom obtain_pick_item_pose.py
```
to obtain the pickup pose for the cricket ball in the scene. Feel
free to change the name of the object in the script located in the debug func.

It start running and will continuously print the pickup pose in the terminal. It will also publish the pickup pose
plus the location of the object to `/locobot/estimated_pickup_pose` and
`/debug/obj_loc` to aid visualization and debugging in rviz.

In Rviz load the custom configuration file named `pickup_visualize.rviz` to get the topic loaded.


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

## Pick and Place
### Run the perception stack launchfile
```
roslaunch interbotix_xslocobot_perception xslocobot_perception.launch robot_model:=locobot_wx200
```

### Run the Object Detection and pick/place python code
```
roscd locobot_custom
cd scripts
python3 pick_place_no_armtag.py
```

It will then pickup every detected object in the camera view, raise it up a few inches, and drop it.


## Integrated Navigation, Perception, and Moveit stack
```
roslaunch locobot_custom nav_moveit_perception.launch
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

### Arm Issue, such as Can’t find DYNAMIXEL ID
If the robot cannot find Dynamixel ID of some of the motors, it could be that the robot was in a position that probably broke one of the wires. It could also be that one of the servo motors on the arm is malfunctioning. To solve the problem, run the Dynamixel Wizard 2 diagnosis tool, and see if the motors can be detected. The Tufts locobot alreadu have it installed at `/home/locobot/ROBOTIS/DynamixelWizard2/DynamixelWizard2.sh`. Run this command and a window will pop up, allowing you to test connections to the motors.

For more debugging help or other issues, see the official documentation [here](https://docs.trossenrobotics.com/interbotix_xsarms_docs/troubleshooting.html#can-t-find-dynamixel-id)
