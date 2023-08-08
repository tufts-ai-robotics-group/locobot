#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from universal_move_arm import LBMoveIt
from locobot_custom.srv import Place

def handle_place(req):
    # Initialize the arm and gripper group using the LBMoveIt class
    arm_group = LBMoveIt(group="arm")
    gripper_group = LBMoveIt(group="gripper")
    
    # Move the arm to the desired pose
    success = arm_group.go_to_pose_goal(req.target.pose)

    if not success:
        return {"success": False, "message": "Failed to move arm to target pose"}

    # Open the gripper to release the object
    gripper_group.open_gripper()

    return {"success": True, "message": "Successfully placed object at target pose"}

def place_server():
    rospy.init_node('place_service')
    s = rospy.Service('place', Place, handle_place)
    rospy.spin()

if __name__ == "__main__":
    place_server()
