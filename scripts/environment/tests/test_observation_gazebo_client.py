#!/usr/bin/env python3
import rospy
from locobot_custom.srv import GazeboObservationService

def test_gazebo_observation_service():
    rospy.init_node('test_gazebo_observation_service_client')
    rospy.wait_for_service('gazebo_observation')
    
    try:
        gazebo_observation_service = rospy.ServiceProxy('gazebo_observation', GazeboObservationService)
        response = gazebo_observation_service()
        
        print("Occupancy Grid:", response.occupancy_grid)
        print("Relative Locations X:", response.relative_locations_x)
        print("Relative Locations Y:", response.relative_locations_y)
        print("Relative Orientations:", response.relative_orientations)
        
    except rospy.ServiceException as e:
        print("Service call failed:", e)

if __name__ == "__main__":
    test_gazebo_observation_service()
