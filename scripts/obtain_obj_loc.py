#!/usr/bin/env python3
##################################################################
# obtain_obj_loc.py
# 
# by GPT
#
# listens to the model states topic and gives the location of the desired object
#
##################################################################

import rospy
from gazebo_msgs.msg import ModelStates
import sys

def create_callback(obj_name):
    def callback(msg):
        if obj_name is None:
            print("Objects:")
            print(",  ".join(msg.name))
            print()
            print()
            return
        # Find the index of the object in the model list
        index = msg.name.index(obj_name)

        # Extract position and orientation
        position = msg.pose[index].position
        orientation = msg.pose[index].orientation

        # Print the position and orientation
        print("Position: x={}, y={}, z={}".format(position.x, position.y, position.z))
        print("Orientation: x={}, y={}, z={}, w={}".format(orientation.x, orientation.y, orientation.z, orientation.w))
    return callback

if __name__ == "__main__":
    if len(sys.argv) < 2:
        obj_name = None
        print("No object specified. Will Print Object name list")
    else:
        obj_name = sys.argv[1]
    rospy.init_node('object_pose_listener')
    rospy.Subscriber('/gazebo/model_states', ModelStates, create_callback(obj_name))
    rospy.spin()
