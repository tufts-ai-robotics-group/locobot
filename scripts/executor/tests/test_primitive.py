#!/usr/bin/env python3

import rospy
from locobot_custom.srv import PrimitiveBase

def test_primitive_action_server():
    rospy.init_node('test_primitive_action_client')
    
    # Wait for the service to become available
    rospy.wait_for_service('primitive_action_service')
    
    # Create a service client
    try:
        service_client = rospy.ServiceProxy('primitive_action_service', PrimitiveBase)
        
        # Test move_forward
        response = service_client("move_forward", 1.0)  # Move forward by 1 meter
        print(f"Move Forward Response: Success: {response.success}, Message: {response.message}")
        
        # Test turn_left
        response = service_client("turn_left", 0.785398)  # Turn left by 45 degrees (0.785398 radians)
        print(f"Turn Left Response: Success: {response.success}, Message: {response.message}")
        
        # Test turn_right
        response = service_client("turn_right", 0.785398)  # Turn right by 45 degrees (0.785398 radians)
        print(f"Turn Right Response: Success: {response.success}, Message: {response.message}")
        
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

if __name__ == '__main__':
    test_primitive_action_server()