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
from geometry_msgs.msg import TransformStamped, PoseStamped, Quaternion
from gazebo_msgs.msg import ModelStates
from tf.transformations import quaternion_from_euler
import tf2_ros
import tf2_geometry_msgs

ARM_RELATIVE_LOC = [0.037943, 0, 0.139]

    

WORLD_TF = "map"
LOCOBOT_TF = "locobot/arm_base_link"

class PickUpPoseCalculator:
    # make sure a node is already initalized
    def __init__(self, object_name):
        self.object_name = object_name
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # Initialize variables to store the object pose and LoCoBot pose
        self.model_states = None
        self.object_pose = None

        def model_state_callback(msg):
            if object_name in msg.name:
                self.model_states = msg
            if object_name in msg.name:
                index = msg.name.index(object_name)
                self.object_pose = PoseStamped(pose=msg.pose[index])
                self.object_pose.header.stamp = rospy.Time.now()
                self.object_pose.header.frame_id = object_name

        # rospy.init_node('pose_listener')

        # Subscribe to the Gazebo model_states topic to get the object and LoCoBot poses
        rospy.Subscriber('/gazebo/model_states', ModelStates, model_state_callback)

    def get_pose(self):
        rate = rospy.Rate(10.0)
        # Wait for the poses to be obtained
        while self.model_states is None:
            rate.sleep()

        # Wait for the transform to be obtained
        while True:
            try:
                target_transform: TransformStamped = self.tf_buffer.lookup_transform(LOCOBOT_TF, WORLD_TF, rospy.Time(0))
                target_pose = tf2_geometry_msgs.do_transform_pose(self.object_pose, target_transform).pose
                target_pose.orientation = Quaternion(*quaternion_from_euler(ai=0, aj=0.8, ak=0))
                return target_pose
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rate.sleep()


if __name__ == "__main__":
    rospy.init_node("pose_listener")
    ball_pose_calculator = PickUpPoseCalculator("cricket_ball")
    while True:
        print("---------")
        print(ball_pose_calculator.get_pose())
        rospy.sleep(1)
    rospy.spin()
