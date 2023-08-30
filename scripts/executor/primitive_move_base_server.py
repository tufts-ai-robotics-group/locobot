#!/usr/bin/env python3
import sys
import rospy
from geometry_msgs.msg import Twist
from locobot_custom.srv import PrimitiveBase, PrimitiveBaseResponse

# Global variable for publisher
cmd_vel_pub = None

def handle_primitive_action(req):
    global cmd_vel_pub
    
    print("Received service call with action:", req.action, "and value:", req.value)
    
    # Create a Twist message
    cmd = Twist()

    # Set the action
    if req.action == "move_forward":
        cmd.linear.x = req.value
        cmd.angular.z = 0
    elif req.action == "turn_left":
        cmd.linear.x = 0
        cmd.angular.z = req.value
    elif req.action == "turn_right":
        cmd.linear.x = 0
        cmd.angular.z = -req.value
    else:
        return PrimitiveBaseResponse(success=False)

    # Start the continuous publishing loop
    rate = rospy.Rate(10)  # 10Hz
    duration = 1.0  # 1 second
    end_time = rospy.Time.now() + rospy.Duration(duration)
    while rospy.Time.now() < end_time:
        cmd_vel_pub.publish(cmd)
        rate.sleep()

    return PrimitiveBaseResponse(success=True)

if __name__ == '__main__':
    # Determine the mode and set the appropriate cmd_vel topic
    mode = rospy.get_param("~mode", "simulation")
    if mode == "simulation":
        cmd_vel_topic = "/locobot/cmd_vel"
    elif mode == "real_robot":
        cmd_vel_topic = "/cmd_vel"
    elif mode == "kobuki_base":
        cmd_vel_topic = "/mobile_base/commands/velocity"
    else:
        sys.exit(f"Invalid mode: {mode}. Valid modes are: simulation, real_robot, kobuki_base.")
    
    rospy.init_node('primitive_action_server_node')
    cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
    
    # Register the service
    service = rospy.Service('primitive_action_service', PrimitiveBase, handle_primitive_action)
    print("Service registered.")
    
    rospy.spin()
