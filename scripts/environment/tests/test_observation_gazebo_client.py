#!/usr/bin/env python3

import rospy
from locobot_custom.srv import GazeboObservation

def test_gazebo_observation_service():
    rospy.init_node('test_gazebo_observation_service_client')
    rospy.wait_for_service('GazeboObservation')
    
    try:
        gazebo_observation_service = rospy.ServiceProxy('GazeboObservation', GazeboObservation)
        response = gazebo_observation_service()
        
        print("Occupancy Grid:", response.occupancy_grid)
        print("Relative Locations X:", response.relative_locations_x)
        print("Relative Locations Y:", response.relative_locations_y)
        print("Relative Orientations:", response.relative_orientations)
        
    except rospy.ServiceException as e:
        print("Service call failed:", e)

if __name__ == "__main__":
    test_gazebo_observation_service()
