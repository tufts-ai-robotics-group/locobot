#!/usr/bin/env python3
##################################################################
# obtain_obj_loc.py
# 
#
# listens to the model states topic and gives the location of the desired object
# and the pick up pose for the robot arm in the cartesian coordinates frame.
#
##################################################################

import rospy
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose, Quaternion, PointStamped
from gazebo_msgs.msg import ModelStates
from tf.transformations import quaternion_from_euler
from moveit_msgs.msg import MoveGroupActionGoal, MoveGroupGoal
import tf2_ros
import tf2_geometry_msgs
import math

ARM_RELATIVE_LOC = [0.037943, 0, 0.139]

    

WORLD_TF = "map"
LOCOBOT_TF = "locobot/base_link"


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
                self.object_pose.header.frame_id = "map"

        # rospy.init_node('pose_listener')

        # Subscribe to the Gazebo model_states topic to get the object and LoCoBot poses
        rospy.Subscriber('/gazebo/model_states', ModelStates, model_state_callback)
    
    
    def world_to_locobot(self, pose_world: PoseStamped) -> PoseStamped:
        target_transform: TransformStamped = self.tf_buffer.lookup_transform(LOCOBOT_TF, WORLD_TF, rospy.Time(0))
        return tf2_geometry_msgs.do_transform_pose(pose_world, target_transform)
    
    def locobot_to_world(self, pose_locobot: PoseStamped) -> PoseStamped:
        target_transform: TransformStamped = self.tf_buffer.lookup_transform(WORLD_TF, LOCOBOT_TF, rospy.Time(0))
        return tf2_geometry_msgs.do_transform_pose(pose_locobot, target_transform)

    def get_pose(self, pitch=1.4, z_offset=0.0, x_offset=0.037943) -> PoseStamped:
        rate = rospy.Rate(10.0)
        # Wait for the poses to be obtained
        while self.model_states is None:
            rate.sleep()

        # Wait for the transform to be obtained
        while True:
            try:
                target_transform: TransformStamped = self.tf_buffer.lookup_transform(LOCOBOT_TF, WORLD_TF, rospy.Time(0))
                target_pose = tf2_geometry_msgs.do_transform_pose(self.object_pose, target_transform)
                # orientation computation
                # 1. aj (pitch) determines the angle between the arm and the ground.
                # 2. ak (yaw) (angle that determines whether the final pose of the gripper 
                #    should go left / right) needs to be determined by arctan because we only have 5 dof.
                # target_pose.pose.position.y = 0 # test
                
                x = target_pose.pose.position.x
                y = target_pose.pose.position.y
                target_pose.pose.orientation = Quaternion(*quaternion_from_euler(ai=0, aj=pitch, ak=math.atan2(y, x)))
                target_pose.pose.position.x += x_offset
                target_pose.pose.position.z += z_offset
                return target_pose
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rate.sleep()

def debug():
    pub = rospy.Publisher("/locobot/estimated_pickup_pose", PoseStamped, queue_size=10)
    pub_object = rospy.Publisher("/debug/obj_loc", PoseStamped, queue_size=10)
    rospy.init_node("pose_listener")
    ball_pose_calculator = PickUpPoseCalculator("ball")
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        pose = ball_pose_calculator.get_pose(pitch=1.4, z_offset=0.05)
        
        # print("---------")
        # print(pose)

        # publish to rviz
        pub.publish(ball_pose_calculator.locobot_to_world(pose))
        pub_object.publish(ball_pose_calculator.object_pose)
        rate.sleep()

if __name__ == "__main__":
    debug()
