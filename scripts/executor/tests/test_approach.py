#!/usr/bin/env python3

# for testing run the following command:
# rosrun locobot_custom/locobot test_approach.py
import rospy
from locobot_custom.srv import Approach

def test_approach_service():
    rospy.wait_for_service('approach')

    try:
        approach_service = rospy.ServiceProxy('approach', Approach)
        response = approach_service("bin")  # Replace "" with your target

        print("Service call was a ", "success" if response.success else "failure")
        print("Info: ", response.info)

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

if __name__ == "__main__":
    # Comment out the following line when you want to run the service
    test_approach_service()
