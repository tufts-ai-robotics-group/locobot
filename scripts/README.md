# Grasp Pose Detection and Grasping for Locobot
This README describes the details of the Grasping software implemented on
[Locobot-WX200](https://www.trossenrobotics.com/locobot-wx200.aspx) -- 5 DOF arm with Create 3 from iRobot Base.

In summary, it mplements **GPD** software that can detect the grasping poses (from **3D Point Clouds**) and,
solves for **Inverse Kinematics** and Plans using **MoveIt** to execute them as actions. 
The stack also implements **YOLO** object detection software to detect and generate bounding boxes on camera images.

Actions: 
    ```
    grasp(?o1), place(?o1 ?o2)
    ```

#### Tested in 
Ubuntu 20.04.6 LTS; ROS Noetic

Dependencies
>darknet_ros: 1.1.5</BR>
>GPD 2.0.0</BR>
>IKPy 3.3.4</BR>
>Moveit 1.1.12</BR>

### Installation

Each of the software used contains installation instructions in their respective GitHub. Here is a list of all of the components required:

* [GPD_ROS](https://github.com/atenpas/gpd_ros) We are using a forked version of this repository: [GPD_ROS-forked](https://github.com/goelshivam1210/gpd_ros)
    * GPD_ROS requires the GPD software whose forked version we use lives here [GPD-forked](https://github.com/goelshivam1210/gpd)
* [YOLO ROS](https://github.com/leggedrobotics/darknet_ros)
* [IKPy](https://github.com/Phylliade/ikpy)
* [Moveit](https://github.com/ros-planning/moveit)


### Run
###### SSH into the robot

```bash
ssh -X username@<IP_address_of_robot>
```

* Mulip Locobot
    ```bash
    ssh -X locobot@10.0.60.2
    ```

######  Run the perception stack
In a terminal

```bash
roslaunch locobot_custom nav_moveit_perception.launch
```

In another terminal
  * Run GPD_pick_and_place.py script

``` bash
 rosrun locobot_custom GPD_pick_and_place.py 
```

## Params

Please go into the ```GPD_pick_and_place.py``` script and adjust the parameters at the top of the screen. These are labeled according to what they control and should be filled out appropriately. Additionally, some are dependent on the components being used for instance one of the parameters is for the tuning parameters of the camera and has instructions on how to find those.

You should also go through and change the parameters in ```ros_eigen_params.cfg``` in the GPD software under the ```cfg``` folder. This contains many parameters dependent on your robot and is used to tune the software to your liking. 

## Troubleshooting

Ensure ``GPD_pick_and_place``` is subscribed to ```/locobot/camera/depth/color/points``` topic.

To rebuild, **delete** `devel` and `build` from `interbotix_ws` & **delete** `build` from the `GPD` directory, then ```catkin_make```.
This might fail, but you need to do this to remake the `build` & `devel` folders enough where GPD can install correctly. From there, you can follow the installation instructions on the GPD GitHub and then ```catkin_make``` your `interbotix_ws` and have it build successfully.

## Future Improvements

* In the Mulip Locobot, the current state GPD is not that accurate as the software generates grasps all around the object, and many of these grasps are simply unreachable. Part of this is because GPD was trained using a 6-degree-of-freedom arm and not a 5-degree-of-freedom arm. To counteract this, we use some basic filtering methods (filtering yaw and other heuristics, check code). Still, these methods must be improved and tuned over time to ensure that grasps that will be properly reachable will be passed on to the executor. 

* The robot has issues with processing times in YOLO and performing IK on potential grasps sent by GPD efficiently. The YOLO process can be sped up by processing it on an external Nvidia Jetson as they take advantage of its GPU acceleration. IK speed could potentially be improved by using an external IK solver library IKPy. A function utilizing this library can be found in the program. Needs testing.

* One of the other issues is that the YOLO aspect of the program takes a point which is not close enough to the point cloud, meaning that GPD can not figure out the object it is referring to. This prevents us from using YOLO properly but could potentially be resolved in the future with careful tuning and calibration of the camera.

* Should program to, instead of filter yaw, make the robot move to a position where it will be graspable. This means communicating with the Roomba base. Eventually, utilizing the navigation task.

## Current major issues
Upon deleting cloud pointer results in double free memory https://github.com/atenpas/gpd_ros/issues/12. For now, we have gotten rid of the delete command, which is causing ram usage to increase over time, but with how we plan to use it (Run GPD only before the robot wants to grasp), this shouldn't be a problem but should be addressed soon.

The program also has to send a message to GPD 2 times with a bit of time between each sent message. We can't tell why this is, as when we ran the program without waiting between messages that were sent, it would take 8 sent messages for GPD to receive one.

## Questions/Issues
For questions and issues, please contact: </BR>
Andrew Esteves @ esteves@hartford.edu </BR>
Shivam Goel @ shivam.goel@tufts.edu
