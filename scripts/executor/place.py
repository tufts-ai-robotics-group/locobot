#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from universal_move_arm import LBMoveIt

def place_callback(data):
    # Initialize the arm and gripper group using the LBMoveIt class
    arm_group = LBMoveIt(group="arm")
    gripper_group = LBMoveIt(group="gripper")
    
    # Move the arm to the desired pose
    arm_group.go_to_pose_goal(data.pose)

    # Open the gripper to release the object
    gripper_group.open_gripper()

rospy.init_node('place_node', anonymous=True)
rospy.Subscriber("/place_pose", PoseStamped, place_callback)
rospy.spin()