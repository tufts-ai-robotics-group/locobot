#!/usr/bin/env python3
##################################################################
# obtain_obj_loc.py
# 
# by GPT
#
# listens to the model states topic and gives the location of the desired object
#
##################################################################

# import rospy
# from gazebo_msgs.msg import ModelStates
# import sys

# def create_callback(obj_name):
#     def callback(msg):
#         if obj_name is None:
#             print("Objects:")
#             print(",  ".join(msg.name))
#             print()
#             print()
#             return
#         # Find the index of the object in the model list
#         index = msg.name.index(obj_name)

#         # Extract position and orientation
#         position = msg.pose[index].position
#         orientation = msg.pose[index].orientation

#         # Print the position and orientation
#         print("Position: x={}, y={}, z={}".format(position.x, position.y, position.z))
#         print("Orientation: x={}, y={}, z={}, w={}".format(orientation.x, orientation.y, orientation.z, orientation.w))
#     return callback

# if __name__ == "__main__":
#     if len(sys.argv) < 2:
#         obj_name = None
#         print("No object specified. Will Print Object name list")
#     else:
#         obj_name = sys.argv[1]
#     rospy.init_node('object_pose_listener')
#     rospy.Subscriber('/gazebo/model_states', ModelStates, create_callback(obj_name))
#     rospy.spin()


import rospy
import math
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def calculate_pickup_pose(object_pose, locobot_pose):
    print(object_pose)
    print()
    print(locobot_pose)
    # Calculate relative position
    relative_position = Point()
    relative_position.x = object_pose.position.x - locobot_pose.position.x
    relative_position.y = object_pose.position.y - locobot_pose.position.y
    relative_position.z = object_pose.position.z - locobot_pose.position.z + 0.2

    # Calculate relative orientation
    locobot_orientation = (
        locobot_pose.orientation.x,
        locobot_pose.orientation.y,
        locobot_pose.orientation.z,
        locobot_pose.orientation.w
    )
    object_orientation = (
        object_pose.orientation.x,
        object_pose.orientation.y,
        object_pose.orientation.z,
        object_pose.orientation.w
    )
    (roll_locobot, pitch_locobot, yaw_locobot) = euler_from_quaternion(locobot_orientation)
    (roll_object, pitch_object, yaw_object) = euler_from_quaternion(object_orientation)
    relative_yaw = yaw_object - yaw_locobot

    # Adjust relative pose for picking up
    offset_x = 0.1  # Adjust the x-coordinate offset for picking up
    offset_z = 0.05  # Adjust the z-coordinate offset for picking up
    relative_position.x += offset_x
    relative_position.z += offset_z

    # Convert relative orientation back to quaternion
    relative_orientation = quaternion_from_euler(0.0, 0.7, relative_yaw)

    # Create the pickup pose in the robot's coordinate frame
    pickup_pose = Pose()
    pickup_pose.position = relative_position
    pickup_pose.orientation.x = relative_orientation[0]
    pickup_pose.orientation.y = relative_orientation[1]
    pickup_pose.orientation.z = relative_orientation[2]
    pickup_pose.orientation.w = relative_orientation[3]

    return pickup_pose


class PickUpPoseCalculator:
    # make sure a node is already initalized
    def __init__(self, object_name):
        # object_name = "cricket_ball"  # Name of the object you want to pick up

        # Initialize variables to store the object pose and LoCoBot pose
        self.object_pose = None
        self.locobot_pose = None

        def object_pose_callback(msg):
            if object_name in msg.name:
                index = msg.name.index(object_name)
                self.object_pose = msg.pose[index]

        def locobot_pose_callback(msg):
            index = msg.name.index("locobot")
            self.locobot_pose = msg.pose[index]  # Assuming the LoCoBot's pose is the second element in the model_states message

        # rospy.init_node('pose_listener')

        # Subscribe to the Gazebo model_states topic to get the object and LoCoBot poses
        rospy.Subscriber('/gazebo/model_states', ModelStates, object_pose_callback)
        rospy.Subscriber('/gazebo/model_states', ModelStates, locobot_pose_callback)

    def get_pose(self):
        # Wait for the poses to be obtained
        while self.object_pose is None or self.locobot_pose is None:
            rospy.sleep(0.1)

        # Calculate the pickup pose
        pickup_pose = calculate_pickup_pose(self.object_pose, self.locobot_pose)

        # Print the pickup pose
        print("Pickup Pose:")
        print("Position: x={}, y={}, z={}".format(pickup_pose.position.x, pickup_pose.position.y, pickup_pose.position.z))
        print("Orientation: x={}, y={}, z={}, w={}".format(pickup_pose.orientation.x, pickup_pose.orientation.y, pickup_pose.orientation.z, pickup_pose.orientation.w))
        return pickup_pose


if __name__ == "__main__":
    rospy.init_node("pose_listener")
    ball_pose_calculator = PickUpPoseCalculator("cricket_ball")
    ball_pose_calculator.get_pose()
    rospy.spin()
