#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from universal_move_arm import LBMoveIt
from locobot.srv import Grasp

def handle_grasp(req):
    # Initialize the arm and gripper group using the LBMoveIt class
    arm_group = LBMoveIt(group="arm")
    gripper_group = LBMoveIt(group="gripper")
    
    # Move the arm to the desired pose
    success = arm_group.go_to_pose_goal(req.target.pose)

    if not success:
        return {"success": False, "message": "Failed to move arm to target pose"}

    # Close the gripper to grasp the object
    gripper_group.close_gripper()

    return {"success": True, "message": "Successfully grasped object at target pose"}

def grasp_server():
    rospy.init_node('grasp_service')
    s = rospy.Service('grasp', Grasp, handle_grasp)
    rospy.spin()

if __name__ == "__main__":
    grasp_server()
