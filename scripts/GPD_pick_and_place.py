#!/usr/bin/env python3

import rospy
import math
import time
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import modern_robotics as mr
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose, Quaternion, PointStamped
import moveit_commander
from moveit_msgs.msg import MoveGroupActionGoal, MoveGroupGoal
from interbotix_xs_modules.locobot import InterbotixLocobotXS
import interbotix_common_modules.angle_manipulation as ang
from geometry_msgs.msg import Point, Vector3, PoseStamped, Quaternion
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32, Int64
import sensor_msgs.point_cloud2 as pc2
from gpd_ros.msg import GraspConfig, GraspConfigList, CloudSources, CloudSamples
from tf.transformations import quaternion_from_matrix
from visualization_msgs.msg import MarkerArray

x_view = -0.008972 # Coordinates of camera
y_view = -0.015818
z_view = 0.000003
topic_PointCloud = "/locobot/camera/depth/color/points" # Topic of the point cloud
coordinates_done = False
grasp_info_done = False
publishing_latch = False

ARM_TF = "locobot/arm_base_link"
LOCOBOT_TF = "locobot/camera_color_optical_frame"

def publish_cloud_samples():
    cloud_samples_msg = CloudSamples() # Create a new CloudSamples message
    cloud_sources_msg = create_cloud_sources()  # Create a new CloudSources message
    cloud_samples_msg.cloud_sources = cloud_sources_msg

    #samples = (Point(0.1, 0.2, 0.3),Point(0.4, 0.5, 0.6),Point(0.7, 0.8, 0.9)) 
    #cloud_samples_msg.samples = samples
    pub_cloud.publish(cloud_samples_msg) # Publish the message
    rospy.sleep(3)
    pub_cloud.publish(cloud_samples_msg) # Publish the message
    print("Cloud Samples message published...")
    publishing_latch = True

def create_cloud_sources():
    cloud_sources_msg = CloudSources()
    cloud_sources_msg.cloud = rospy.wait_for_message(topic_PointCloud, PointCloud2, timeout=10) # Fills in point cloud
    
    camera_sources = [Int64(2)] # Assigns camera source
    cloud_sources_msg.camera_source = camera_sources

    view_point = [Point(np.float64(x_view),np.float64(y_view),np.float64(z_view))] # Coordinates of camera
    cloud_sources_msg.view_points = view_point
    print("Cloud Sources message created...")
    return cloud_sources_msg

def callback_coordinates(msg):
    pose = PoseStamped()
    print("Information recieved...")
    
    marker = msg.markers[3] # Assuming the first marker in the array contains the quaternion information
    quaternion = (                # Extract the quaternion components from the marker message
         marker.pose.orientation.x,
         marker.pose.orientation.y,
         marker.pose.orientation.z,
         marker.pose.orientation.w
     )
    pose.pose.orientation = Quaternion(*quaternion)  # Set the quaternion for the pose
    
    pose.pose.position = marker.pose.position
    x, y, z = marker.pose.position.x, marker.pose.position.y, marker.pose.position.z # Store coordinates for use with arm
    print("Information processed...")

    publish_pose(pose, pub_coordinates) # Publishes pose

    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    rospy.sleep(2)

    target_transform: TransformStamped = buffer.lookup_transform(ARM_TF, LOCOBOT_TF, rospy.Time(0))
    target_pose = tf2_geometry_msgs.do_transform_pose(pose, target_transform)

    roll, pitch, yaw = euler_from_quaternion(target_pose.pose.orientation.x,
        target_pose.pose.orientation.y,
        target_pose.pose.orientation.z,
        target_pose.pose.orientation.w)

    print("x = {} y = {} z = {} roll = {} pitch = {} yaw = {}".format(x, y, z, roll, pitch, yaw))
    move_arm(target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z, roll, pitch, yaw)
    print("Movement completed...")
    coordinates_done == True
    
def callback_grasp_info(msg):
    print("This is a temporary message")

def publish_pose(pose,publisher):
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "locobot/camera_color_optical_frame"
    publisher.publish(pose)
    print("Published...")

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

def move_arm(x,y,z,roll,pitch,yaw):
    print("Moving arm...")
    bot.arm.set_ee_pose_components(x=0.3, z=0.2)
    bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.05, pitch=0.8)
    time.sleep(0.2)
    bot.arm.set_ee_pose_components(x=x, y=y, z=z, pitch=0.8)
    time.sleep(0.2)
    bot.gripper.close()
    time.sleep(0.2)

    bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.05, pitch=0.5)
    time.sleep(0.2)
    bot.arm.set_ee_pose_components(x=0.4, y=-0.3, z=0.2, moving_time=1.5)
    time.sleep(0.2)
    bot.gripper.open()
    time.sleep(0.2)
    
    bot.arm.set_ee_pose_components(x=0.3, z=0.2, moving_time=1.5)
    bot.arm.go_to_sleep_pose()
    time.sleep(0.1)

if __name__ == "__main__":  
    bot = InterbotixLocobotXS("locobot_wx200", arm_model="mobile_wx200")
    bot.camera.pan_tilt_move(0, 0.75) # Set camera tilt angle
    bot.arm.set_ee_pose_components(x=0.3, z=0.2) # safe pose
    bot.arm.go_to_sleep_pose() # move the arm to the sleep pose

    rospy.Subscriber('detect_grasps/plot_grasps', MarkerArray, callback_coordinates, queue_size=100)
    rospy.Subscriber('detect_grasps/clustered_grasps', GraspConfigList, callback_grasp_info, queue_size=100)

    pub_coordinates = rospy.Publisher('grasp_pose', PoseStamped, queue_size=1000)
    pub_cloud = rospy.Publisher('cloud_stitched', CloudSamples, queue_size=10)
    print("Subscribers and publishers made...")

    publish_cloud_samples()
    print("Point cloud sent...")
      # pub_claw_width = rospy.Publisher('claw_width', PoseStamped, queue_size=1000)
    while not rospy.is_shutdown():
        if coordinates_done == True:
            rospy.signal_shutdown("Node shutting down")
        rospy.spin()
       