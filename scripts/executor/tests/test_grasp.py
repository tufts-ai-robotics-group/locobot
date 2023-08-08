#!/usr/bin/env python3

import rospy
from locobot_custom.srv import Grasp
from geometry_msgs.msg import PoseStamped

def pickup_pose_callback(data):
    global pickup_pose
    pickup_pose = data

def test_grasp_service():
    global pickup_pose
    rospy.wait_for_service('grasp')
    rospy.Subscriber("/locobot/estimated_pickup_pose", PoseStamped, pickup_pose_callback)

    # Wait for the pickup_pose to be updated
    rospy.sleep(2)

    try:
        grasp = rospy.ServiceProxy('grasp', Grasp)
        response = grasp(pickup_pose)

        print("Service call was a ", "success" if response.success else "failure")
        print("Message: ", response.message)

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

if __name__ == "__main__":
    rospy.init_node('test_grasp_node')
    global pickup_pose
    pickup_pose = PoseStamped() # Default initialization
    test_grasp_service()
