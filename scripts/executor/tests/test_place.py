# for testing run the following command:
# rosrun locobot_custom/locobot test_place.py


import rospy

def test_place_service():
    rospy.wait_for_service('place')

    try:
        place = rospy.ServiceProxy('place', Place)
        pose = PoseStamped()  # Fill this with your desired pose
        response = place(pose)

        print("Service call was a ", "success" if response.success else "failure")
        print("Message: ", response.message)

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

if __name__ == "__main__":
    # Comment out the following line when you want to run the service
    test_place_service()
