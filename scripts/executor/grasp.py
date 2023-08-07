#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from universal_move_arm import LBMoveIt

def grasp_callback(data):
    # Initialize the arm and gripper group using the LBMoveIt class
    arm_group = LBMoveIt(group="arm")
    gripper_group = LBMoveIt(group="gripper")
    
    # Move the arm to the desired pose
    arm_group.go_to_pose_goal(data.pose)

    # Close the gripper to grasp the object
    gripper_group.close_gripper()

rospy.init_node('grasp_node', anonymous=True)
rospy.Subscriber("/grasp_pose", PoseStamped, grasp_callback)
rospy.spin()
