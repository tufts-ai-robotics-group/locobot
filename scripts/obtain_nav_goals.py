#!/usr/bin/env python3
##################################################################
# obtain_nav_goals.py
# 
# by GPT
#
# listens to the move base goal topic and print the goals
# assigned in rviz
#
##################################################################


import rospy
from geometry_msgs.msg import PoseStamped

def pose_callback(msg):
    position = msg.pose.position
    orientation = msg.pose.orientation
    rospy.loginfo("Position: x={}, y={}, z={}".format(position.x, position.y, position.z))
    rospy.loginfo("Orientation: x={}, y={}, z={}, w={}".format(orientation.x, orientation.y, orientation.z, orientation.w))

if __name__ == '__main__':
    rospy.init_node('pose_listener', anonymous=True)
    rospy.Subscriber('/locobot/move_base_simple/goal', PoseStamped, pose_callback)
    rospy.spin()


'''
Table to pickup objects from

osition: x=-3.171874761581421, y=-1.5111654996871948, z=0.0
[INFO] [1688430300.462860, 4952.476000]: Orientation: x=0.0, y=0.0, z=-0.8724284088357078, w=0.48874192725445104
'''
