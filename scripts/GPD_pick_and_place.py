#!/usr/bin/env python3

# By Andrew Esteves
# Please email esteves@hartford.edu with any questions
# This code will take bounding box from YOLO ROS and use it to find a point on a desired object. It will then feed a 
# point cloud and that point into GPD so that it only grasps that object. The program will then recieve that grasp information
# and process it using inverse kinematics to make sure that the point is graspable by the arm before grasping the object infront of it. 

import rospy
import math
import time
import numpy as np
import sys
import tf2_ros
import tf2_geometry_msgs
import cv_bridge
import modern_robotics as mr
from tf.transformations import quaternion_matrix
from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose, Quaternion, PointStamped
from moveit_msgs.msg import MoveGroupActionGoal, MoveGroupGoal,RobotState
import moveit_commander
from control_msgs.msg import JointTrajectoryControllerState
#from moveit_msgs.srv import GetMotionPlan, GetMotionPlanRequest
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from interbotix_xs_modules.locobot import InterbotixLocobotXS
import interbotix_common_modules.angle_manipulation as ang
from geometry_msgs.msg import Point, Vector3, PoseStamped, Quaternion
from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import Float32, Int64, Header
import sensor_msgs.point_cloud2 as pc2
from gpd_ros.msg import GraspConfig, GraspConfigList, CloudSources, CloudSamples
from tf.transformations import quaternion_from_matrix
from visualization_msgs.msg import MarkerArray

x_view = -0.008972 # Coordinates of camera
y_view = -0.015818
z_view = 0.000003
topic_PointCloud = "/locobot/camera/depth/color/points" # Topic of the point cloud
coordinate_array = []
grasp_info_array = []
coordinates_done = False
grasp_info_done = False

ARM_TF = "locobot/arm_base_link"
LOCOBOT_TF = "locobot/camera_color_optical_frame"

def publish_cloud_samples(): # Creates cloud samples message that is fed into GPD
    cloud_samples_msg = CloudSamples() # Create a new CloudSamples message
    cloud_sources_msg = create_cloud_sources()
    cloud_samples_msg.cloud_sources = cloud_sources_msg # Stores cloud sources message in cloud samples message

    #samples = (Point(0.1, 0.2, 0.3),Point(0.4, 0.5, 0.6),Point(0.7, 0.8, 0.9)) # Adds point on object YOLO recognizes making GPD grasp that robot
    #cloud_samples_msg.samples = samples
    pub_cloud.publish(cloud_samples_msg)
    rospy.sleep(3)
    pub_cloud.publish(cloud_samples_msg) # This publishes the message a second time as we had issues only sending one message or sending 2 rapidly
    print("Cloud Samples message published...") # Not sure why GPD does that but this temporary solution has served us well

def create_cloud_sources(): # Creates cloud sources message that is stored in cloud samples message
    cloud_sources_msg = CloudSources()
    cloud_sources_msg.cloud = rospy.wait_for_message(topic_PointCloud, PointCloud2, timeout=10) # Gets point cloud from camera

    cloud_sources_msg.view_points = [Point(np.float64(x_view),np.float64(y_view),np.float64(z_view))] # Coordinates of camera
    print("Cloud Sources message created...")
    return cloud_sources_msg

def callback_coordinates(msg): # Stores grasp coordinate information from GPD
    pose = Pose()
    print("Coordinates recieved...")
    global coordinate_array
    global coordinates_done

    i = 0
    while i < (len(msg.markers)/4): # Message has 4 markers for each part of end effector. We only need to look at oe part per pose
        pose = PoseStamped()
        marker = msg.markers[3+(i*4)] # Iterate through grasp info and only look at last part which is the base of end effector
        quaternion = (                # Extract the quaternion components from the marker message
            marker.pose.orientation.x,
            marker.pose.orientation.y,
            marker.pose.orientation.z,
            marker.pose.orientation.w
        )
        pose.pose.orientation = Quaternion(*quaternion) 
        pose.pose.position = marker.pose.position
        coordinate_array.append(pose)
        i = i+1

    print("Coordinates processed...")
    coordinates_done = True
    
def callback_grasp_info(msg): # Stores grasp score in an array
    print("Grasp info recieved...")
    global grasp_info_array
    global grasp_info_done
    i = 0
    while i < len(msg.grasps):
        score = msg.grasps[i].score
        grasp_info_array.append(score)
        i = i+1
    print("Grasp information processed...")
    grasp_info_done = True

def IK_verification(pose): # Inverse kinematics function to esure grasp is reachable by arm
    arm_group = moveit_commander.MoveGroupCommander(robot_description = "/locobot/robot_description", name = "interbotix_arm", ns = "locobot")
    
    # computing the orientations
    roll,pitch,yaw = euler_from_quaternion(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w)
    yaw = math.atan2(pose.pose.position.y,pose.pose.position.x) # Manually calculates yaw as nature of 5 degree arm makes it hard to control
    x,y,z,w = get_quaternion_from_euler(0, 0, yaw)
    pose.pose.orientation.x = x
    pose.pose.orientation.y = y
    pose.pose.orientation.z = z
    pose.pose.orientation.w = w  # sorry for the crude way
   
    arm_group.set_pose_target(pose, end_effector_link="locobot/gripper_link") # Set target pose
    error_code, _, planning_time , _ = arm_group.plan() # Plan motion and return whether point is viable or not/calculation time
    print("Plan success: " + str(error_code) + " Planning time: " + str(planning_time))
    
    # Performs transfromaton from one link to other
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer) # Listens to tf to get transformaton information necessary
    rospy.sleep(2) # Provides time for buffer to collect data
    target_transform: TransformStamped = buffer.lookup_transform(ARM_TF, LOCOBOT_TF, rospy.Time(0))
    target_pose = tf2_geometry_msgs.do_transform_pose(pose, target_transform)

    
    
    if error_code == True:
        print(target_pose)
        move_arm(target_pose)
    # Set other planning parameters if needed
    #request.ik_request.timeout = rospy.Duration(5.0)  # Maximum time to wait for response

    # try:
    #     # Call the service and store the response
    #     response = moveit_ik_service(request)

    #     # Print the result
    #     if response.error_code.val == response.error_code.SUCCESS:
    #         print("IK solution found!")
    #         joint_positions = response.solution.joint_state.position
    #         #joint_positions = arm_group.get_current_joint_values()
    #         return True, joint_positions
    #         # Process the IK solution in 'response.solution' if needed
    #     else:
    #         print("Failed to find IK solution. Error code:") #response.error_code.val)
    #         return False, []
    # except rospy.ServiceException as e:
    #     print("Service call failed:", e)
    return error_code

def publish_pose(pose,publisher): # Used to publish pose
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "locobot/camera_color_optical_frame"
    publisher.publish(pose)
    print("Published...")

def euler_from_quaternion(x, y, z, w): # Converts quaternion coordinates to euler coordinates
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

def get_quaternion_from_euler(roll, pitch, yaw): # Converts euler coordinates to quaternion
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

def move_arm(pose): # Function to pick up and place object
    pub_coordinates.publish(pose)
    
    x,y,z = pose.pose.position.x, pose.pose.position.y, pose.pose.position.z
    roll, pitch, yaw = euler_from_quaternion(pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w)
    
    print("Moving arm...")
    print("x = {} y = {} z = {} roll = {} pitch = {} yaw = {}".format(x, y, z, roll, pitch, yaw))
    bot.arm.set_ee_pose_components(x=0.3, z=0.2)
    bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.05, pitch=0.8) # Moves slightly behind object as to not contact it
    time.sleep(0.2)
    bot.arm.set_ee_pose_components(x=x, y=y, z=z, pitch=0.8) # Move to object to pick up
    time.sleep(0.2)
    bot.gripper.close()
    time.sleep(0.2)

    bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.05, pitch=0.5) # Moves to hard coded object drop point
    time.sleep(0.2)
    bot.arm.set_ee_pose_components(x=0.4, y=-0.3, z=0.2, moving_time=1.5)
    time.sleep(0.2)
    bot.gripper.open()
    time.sleep(0.2)
    
    bot.arm.set_ee_pose_components(x=0.3, z=0.2, moving_time=1.5) # Moves arm to sleep position
    bot.arm.go_to_sleep_pose()
    time.sleep(0.1)

if __name__ == "__main__":  
    bot = InterbotixLocobotXS("locobot_wx200", arm_model="mobile_wx200")
    bot.camera.pan_tilt_move(0, 0.75) # Set camera tilt angle
    
    # Resets arm position to ensure arm is ready to move to any position
    bot.arm.set_ee_pose_components(x=0.3, z=0.2) # Y defaults to 0
    bot.arm.go_to_sleep_pose()

    rospy.Subscriber('detect_grasps/plot_grasps', MarkerArray, callback_coordinates, queue_size=100) # Crate publishers and subscribers
    rospy.Subscriber('detect_grasps/clustered_grasps', GraspConfigList, callback_grasp_info, queue_size=100)
    pub_coordinates = rospy.Publisher('grasp_pose', PoseStamped, queue_size=1000) 
    pub_cloud = rospy.Publisher('cloud_stitched', CloudSamples, queue_size=10)
    print("Subscribers and publishers made...")

    yolo = rospy.wait_for_message("/darknet_ros/bounding_boxes", BoundingBoxes, timeout=10) 
    depth_map = rospy.wait_for_message("/locobot/camera/depth/image_rect_raw", Image, timeout=10)

    x_pixel = round(((yolo.bounding_boxes[0].xmax)/2)+yolo.bounding_boxes[0].xmin) # Finds center of YOLO bounding box/centroid of object
    y_pixel = round(((yolo.bounding_boxes[0].ymax)/2)+yolo.bounding_boxes[0].ymin)

    try:
        cv_bridge_instance = cv_bridge.CvBridge()
        depth_image = cv_bridge_instance.imgmsg_to_cv2(depth_map, desired_encoding="passthrough")
        depth_value = depth_image[y_pixel, x_pixel]  # Find depth of specific pixel
        print(f"Depth value at pixel ({x_pixel}, {y_pixel}): {depth_value}")
        
    except Exception as e:
        print(f"Error: {e}")

    publish_cloud_samples() # Publishes point cloud to GPD
    print("Point cloud sent...")

    while not rospy.is_shutdown():
        x = 0
        while coordinates_done == False or grasp_info_done == False: # Waits for coordinate array and grasp score array are filled
            time.sleep(0.1)
            x = x+1
            if x > 150: # Adds a 15 second time out for the while loop
                print("Error: Was not able to properly recieve and process information")
                break
        
        if coordinates_done == True or grasp_info_done == True:
            x = 0
            success_array = [] # Stores whether grasp suceeds in inverse kinematics test or not
            for grasps in grasp_info_array:
                success = IK_verification(coordinate_array[x])
                success_array.append(success)
                #print(joint_pose)
                x = x + 1
            print("Success: " + str(success_array))
        rospy.signal_shutdown('Exiting ROS node')
    rospy.spin()
       