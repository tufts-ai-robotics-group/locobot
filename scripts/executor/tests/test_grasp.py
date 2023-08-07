# for testing run the following command:
# rosrun locobot_custom/locobot test_grasp.py

import rospy

def test_grasp_service():
    rospy.wait_for_service('grasp')

    try:
        grasp = rospy.ServiceProxy('grasp', Grasp)
        pose = PoseStamped()  # Fill this with your desired pose
        response = grasp(pose)

        print("Service call was a ", "success" if response.success else "failure")
        print("Message: ", response.message)

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

if __name__ == "__main__":
    # Comment out the following line when you want to run the service
    test_grasp_service()
