# Grasp Pose Detection and Grasping for Locobot
#### Tested in 
Ubuntu 20.04.6 LTS; ROS Noetic

Using darknet_ros: 1.1.5
GPD 2.0.0
ikpy 3.3.4
Moveit 1.1.12

### Installation

Each of these softwares use each contain installation instructions in their respective githubs. Here is a list of all of the components required:


* [GPD ROS](https://github.com/atenpas/gpd_ros) We are using a forked version of this repository: [GPD_ROS-forked](https://github.com/goelshivam1210/gpd_ros)
    * GPD ROS requires the GPD software whose forked version we use lives here [GPD-forked](https://github.com/goelshivam1210/gpd)
* [YOLO ROS](https://github.com/leggedrobotics/darknet_ros)
* [IKPy](https://github.com/Phylliade/ikpy)
* [Moveit](https://github.com/ros-planning/moveit)

### Run
First, SSH into the robot

```
ssh -X locobot@10.0.60.2
```

Next, run the perception stack

```
roslaunch locobot_custom nav_moveit_perception.launch
```

Then run the GPD_pick_and_place.py script
```
rosrun locobot_custom GPD_pick_and_place.py 
```

## Params

Please go into the GPD_pick_and_place.py script and adjust the parameters at the top of the screen. These are labeled according to what they control and should be filled out appropriately. Additionally some are dependent on the components being used for instance one of the parameters is for the tuning parameters of the camera and has instructions on how to find those. 

You should also go through and change the parameters in ros_eigen_params.cfg in the GPD software under the cfg folder. This contains many parameters that are dependent on your robot and are also used to tune the software to your liking. 

## Trouble shooting


Make sure GPD_pick_and_place is subscribed to /locobot/camera/depth/color/points

To fully rebuild delete devel and build from interbotix_ws and delete build from the GPD folder then catkin_make. This will fail but you need to do this to remake the build end devel folders enough where GPD can install correctly. From there you can follow the instalation instructions on the GPD github and then catkin_make your interbotix_ws and have it build successfully.

## Future Improvements
In the robots current state GPD is not that accurate as the software generates grasps all around the object and many of these grasps are simply unreachable. Part of this is because GPD was trained using a 6 degree of freedom arm and not a 5 degree of freedom arm. To counteract this I use some basic filtering methods but these methods must be imporved and tuned overtime as to ensure that grasps that will be propperly reachable will be passed on to the arm. 

The robot has issues with processing times in YOLO and performing ik on potential graps sent by GPD efficiently. The YOLO process can be speed up by processing it on an external nvidia jetson as take advantage of it's GPU acceleration. IK speed could potentially be improved by using a library by the name of ikpy. A function ustilizing this library can be found in the program. 

One of the other issues is that the YOLO aspect of the program takes a point which is not close enough to the point cloud meaning that GPD can not figure out the object it is reffering to. This prevents us from using YOLO propperly but could potentially be resolved in the future with careful tuning and calibration of the camera.

Should program to instead of filter yaw make robot move to position where it will be graspable. This means communicating with the roomba base.


## Current major issues
Upon deleting cloud pointer results into double free memory https://github.com/atenpas/gpd_ros/issues/12. For now we have gotten rid of the delete command which is causing ram usage to increase over time but with the way we plan to use it this shouldn't be a problem but should be addressed in the near future.

On another note the program also has to send a meggese to GPD 2 times with a bit of time beteen each sent message. I can't tell why this is as when I ran the program without waiting between messages that  sent it would take 8 sent messages for GPD to recieve one. 


