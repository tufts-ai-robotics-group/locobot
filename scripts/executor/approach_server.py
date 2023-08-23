#!/usr/bin/env python3

import rospy
import math
from locobot_custom.srv import Approach
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib.simple_action_client import SimpleActionClient

# This will be populated from the ROS parameter server
GOAL_LOCATIONS = {}

def initialize_ros_node():
    if not rospy.get_node_uri():
        rospy.init_node('approach_server', anonymous=True)
    global GOAL_LOCATIONS
    try:
        # Retrieve the GOAL_LOCATIONS from the ROS parameter server
        GOAL_LOCATIONS = rospy.get_param("sim_nav_goals")
    except rospy.ROSException as e:
        rospy.logerr("Could not retrieve GOAL_LOCATIONS parameter: %s", e)
        exit(1)

def sanitize_quaternion_for_2d_navigation(orientation):
    q_x = orientation.x
    q_y = orientation.y
    q_z = orientation.z
    q_w = orientation.w
    
    yaw = math.atan2(2.0 * (q_w * q_z + q_x * q_y), 1.0 - 2.0 * (q_y * q_y + q_z * q_z))
    
    orientation.w = math.cos(yaw / 2.0)
    orientation.z = math.sin(yaw / 2.0)
    orientation.x = 0.0
    orientation.y = 0.0
    
    return orientation

def send_goal(goal_name: str):
    client = SimpleActionClient('/locobot/move_base', MoveBaseAction)
    client.wait_for_server()

    pose_dict = GOAL_LOCATIONS[goal_name]
    rospy.loginfo("Sending goal: %s", pose_dict)

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.pose.position = Point(**pose_dict['position'])
    goal.target_pose.pose.orientation = sanitize_quaternion_for_2d_navigation(Quaternion(**pose_dict['orientation']))
    
    client.send_goal(goal)
    
    # Wait for the result
    client.wait_for_result(timeout=rospy.Duration(120))
    
    state = client.get_state()
    if state == 3:  # SUCCEEDED
        rospy.loginfo("Goal reached!")
        return True, "Goal reached!"
    else:
        rospy.logerr("Failed to reach goal!")
        return False, "Failed to reach goal!"

def handle_approach(req):
    rospy.loginfo("Executing approach action towards %s", req.target)

    success, info = send_goal(req.target)

    return {"success": success, "message": info}

if __name__ == "__main__":
    initialize_ros_node()
    rospy.Service('approach', Approach, handle_approach)
    rospy.spin()
