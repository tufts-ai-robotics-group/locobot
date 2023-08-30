#!/usr/bin/env python3

import rospy
from locobot_custom.srv import Grasp, GraspPose
from geometry_msgs.msg import PoseStamped

def pickup_pose_callback(data):
    global pickup_pose_received
    global pickup_pose
    pickup_pose = data
    pickup_pose_received = True

def test_grasp_service():
    global pickup_pose_received
    rospy.wait_for_service('grasp')
    rospy.wait_for_service('compute_and_publish_grasp_pose')
    
    # Subscribe to get the published pose
    rospy.Subscriber("/computed_grasp_pose", PoseStamped, pickup_pose_callback)
    pickup_pose_received = False
    
    try:
        # Call the GraspPose service to trigger publishing of the pickup pose
        grasp_pose_service = rospy.ServiceProxy('compute_and_publish_grasp_pose', GraspPose)
        grasp_pose_response = grasp_pose_service("ball")  # add the object name here
        
        # Check if the service call was successful
        if not grasp_pose_response.success:
            print("Failed to compute grasp pose. Reason:", grasp_pose_response.message)
            return
        
        # Wait until the pickup_pose is updated
        rate = rospy.Rate(10)
        while not pickup_pose_received:
            rate.sleep()
        
        # Call the grasp service
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