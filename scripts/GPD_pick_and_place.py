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
import ikpy.chain
import ikpy.utils.plot as plot_utils
from ikpy.link import OriginLink, URDFLink
from cv_bridge import CvBridge
import modern_robotics as mr
from grasp_utils import generate_intermediate_pose # need to figure out what is wrong with code in this script                                                                            
from tf.transformations import quaternion_matrix
from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose, Quaternion, PointStamped
from moveit_msgs.msg import MoveGroupActionGoal, MoveGroupGoal,RobotState, PositionIKRequest
import moveit_commander
from control_msgs.msg import JointTrajectoryControllerState
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from interbotix_xs_modules.locobot import InterbotixLocobotXS
import interbotix_common_modules.angle_manipulation as ang
from geometry_msgs.msg import Point, Vector3, PoseStamped, Quaternion
from sensor_msgs.msg import PointCloud2, Image, JointState
from std_msgs.msg import Float32, Int64, Header
import sensor_msgs.point_cloud2 as pc2
from gpd_ros.msg import GraspConfig, GraspConfigList, CloudSources, CloudSamples
from tf.transformations import quaternion_from_matrix
from visualization_msgs.msg import MarkerArray
from rospy import Time

# Variables to be set based on task
objects_of_interest = ["dog", "teddy bear"] # Specify objects of interest for YOLO to target
 
# Variables to be set based on robot
## Can be tuned and changed according to the user.
x_view = -0.008972 # Coordinates of camera
y_view = -0.015818
z_view = 0.000003
fx, fy, cx, cy = 604.5020141601562, 604.4981079101562, 320.894287109375, 245.15847778320312  # Camera intrinsics can be gotten through /locobot/camera/aligned_depth_to_color/camera_info topic
topic_PointCloud = "/locobot/camera/depth/color/points" # Topic of the point cloud
ARM_TF = "locobot/arm_base_link"
LOCOBOT_TF = "locobot/camera_color_optical_frame" 
##############################################################################################


coordinate_array = []
grasp_info_array = []
coordinates_done = False
grasp_info_done = False

def yolo_find_object(): 
    '''
    I/P: Takes in the bounding box and depth image
    O/P: returns centroid point on target object
    Description:Gets bouding box from yolo and returns point on target object
    '''
    try:
        # Check if any detected objects are in the specified array
        yolo_msg = rospy.wait_for_message("/darknet_ros/bounding_boxes", BoundingBoxes, timeout=10)
        depth_map = rospy.wait_for_message("/locobot/camera/depth/image_rect_raw", Image, timeout=10)
        
        cv_bridge = CvBridge()
        depth_image = cv_bridge.imgmsg_to_cv2(depth_map, desired_encoding="passthrough")

        detected_objects = yolo_msg.bounding_boxes # Store bounding boxes
        for bbox in detected_objects: # Looks through all bounding boxes to see if any contain object of interest
            if bbox.Class in objects_of_interest:
                center_x = round((bbox.xmin + bbox.xmax) / 2) # Finds center of bounding box
                center_y = round((bbox.ymin + bbox.ymax) / 2)

                # Convert pixel to 3D point in camera frame
                z_camera = depth_image[center_y, center_x]/ 1000.0  # Convert mm to meters
                x_camera = (center_x - cx) * z_camera / fx
                y_camera = (center_y - cy) * z_camera / fy

                point_camera = Point()
                point_camera.x = x_camera
                point_camera.y = y_camera
                point_camera.z = z_camera

                print("Detected object:", bbox.Class)
                print("Center pixel:", center_x, center_y)
                print("Corresponding point in point cloud:", point_camera)
                yolo_pub.publish(point_camera)
                return point_camera
    except Exception as e:
        print("Error:", e)

def publish_cloud_samples():
     # Creates cloud samples message that is fed into GPD

    cloud_samples_msg = CloudSamples() # Create a new CloudSamples message
    cloud_sources_msg = create_cloud_sources()
    cloud_samples_msg.cloud_sources = cloud_sources_msg # Stores cloud sources message in cloud samples message

    #cloud_samples_msg.samples = [point] # Used to send point to GPD to select object to put graps around but camera not calibrated so doesn't work
    pub_cloud.publish(cloud_samples_msg)
    rospy.sleep(3) # workaround for GPD buffer
    pub_cloud.publish(cloud_samples_msg) # This publishes the message a second time as we had issues only sending one message or sending 2 rapidly
    print("Cloud Samples message published...") # Not sure why GPD does that but this temporary solution has served us well

def create_cloud_sources(): # Creates cloud sources message that is stored in cloud samples message
    cloud_sources_msg = CloudSources()
    cloud_sources_msg.cloud = rospy.wait_for_message(topic_PointCloud, PointCloud2, timeout=10) # Gets point cloud from camera

    cloud_sources_msg.view_points = [Point(np.float64(x_view),np.float64(y_view),np.float64(z_view))] # Coordinates of camera
    print("Cloud Sources message created...")
    return cloud_sources_msg

def callback_coordinates(msg): 
    # Stores grasp coordinate information from GPD
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
    
def callback_grasp_info(msg): 
    # Stores grasp score in an array
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

def my_ikpy(pose): 
    # IK function using the python ikpy library which should be able to 
    # parse through data much more quickly
    x_pos,y_pos,z_pos = pose.pose.position.x, pose.pose.position.y, pose.pose.position.z
    roll,pitch,yaw = euler_from_quaternion(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w)
    if -0.524 < yaw < 0.524: # Filtering to ensure yaw is within range graspable to arm
        yaw = math.atan2(pose.pose.position.y,pose.pose.position.x) # Manually calculates yaw as nature of 5 degree arm makes it hard to control

    target_position = [x_pos, y_pos, z_pos]
    target_orientation = [roll, pitch, pitch]

    # Perform inverse kinematics
    target_pose = np.concatenate((target_position, target_orientation))
    joint_angles = my_chain.inverse_kinematics(target_pose)
    reachable = my_chain.is_possible(joint_angles) # Returns boolean on if grasp is possible
    return reachable

def solve_ik(pose): 
    # IK function using compute ik topic
    # ik topic can take in coordinates and perform inverse kinematics to figure out of grasp is reachable
    request = PositionIKRequest()

    joint_state_locobot = rospy.wait_for_message("/locobot/joint_states", JointState, timeout=10)

    request.group_name = "interbotix_arm"
    request.robot_state.joint_state = joint_state_locobot
    request.pose_stamped = pose
    
    request.timeout = rospy.Duration(5, 0)

    try:
        # Call the IK service
        rospy.wait_for_service("/locobot/compute_ik")
        compute_ik_service = rospy.ServiceProxy("/locobot/compute_ik", GetPositionIK)
        response = compute_ik_service(request)
        print(response)

        if response.error_code.val == response.error_code.SUCCESS:
            # IK solution found
            joint_positions = response.solution.joint_state.position
            return True, joint_positions
        else:
            # IK solution not found
            return False, []

    except rospy.ServiceException as e:
        print("Service call failed:", e)
        return False, []

def IK_verification(pose): 
    # Inverse kinematics function to esure grasp is reachable by arm
    # computing the orientations
    roll,pitch,yaw = euler_from_quaternion(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w)
    print("yaw: " + str(yaw))
    if 1.5708 < yaw: # GPD assumes what is behind object because there is only 1 view so we find inverse of yaw if it is behind object
        yaw = 3.13159 - yaw # get the inverse of the yaw
    elif -1.5708 > yaw: # fropm the other side
        yaw = 3.13159 + yaw
    if -0.17 < yaw < 0.17: # This restricts the yaw to what 10 degrees since yaw is determiuned by x,y position of end effector
        yaw = math.atan2(pose.pose.position.y,pose.pose.position.x) # Manually calculates yaw as nature of 5 degree arm makes it hard to control
        x,y,z,w = get_quaternion_from_euler(roll, pitch, yaw)
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
        
        return error_code,yaw
    
        # Set other planning parameters if needed
    else:
        error_code = False
        return error_code,yaw

def euler_from_quaternion(x, y, z, w): 
        # Helper function that converts quaternion coordinates to euler coordinates
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
    yaw = math.atan2(pose.pose.position.y,pose.pose.position.x)

    # Creates aproach position as to not knock object over
    roll = 0 # set to 0 to constrain GPD until we figure out how to filter out incorrect rolls
    object_pose = np.array([x, y, z, roll, pitch, yaw])  # replace with your actual values
    delta = -0.1  # offset
    axis = 0  # 0 for x-axis, 1 for y-axis, 2 for z-axis    
    x_intermidiate,y_intermidiate,z_intermidiate,roll_intermidiate,pitch_intermidiate,yaw_intermidiate = generate_intermediate_pose(object_pose, delta, axis)
    delta_final = 0.03
    # this adjusts the final grasp- poise to get a bit deeper into the object to succesfully grasp it
    x_final, y_final, z_final, roll_final, pitch_final, yaw_final = generate_intermediate_pose(object_pose, delta_final, axis)

    print("Moving arm...")
    print("x = {} y = {} z = {} roll = {} pitch = {} yaw = {}".format(x_intermidiate, y_intermidiate, z_intermidiate, roll_intermidiate, pitch_intermidiate, yaw_intermidiate))
    bot.arm.set_ee_pose_components(x=0.3, z=0.2)
    bot.arm.set_ee_pose_components(x=x_intermidiate, y=y_intermidiate, z=z_intermidiate, roll = roll_intermidiate, pitch = pitch_intermidiate) # Moves slightly behind object as to not contact it
    time.sleep(0.2)
    bot.arm.set_ee_pose_components(x=x_final, y=y_final, z=z_final, roll = roll_final, pitch = pitch_final) # Move to object to pick up
    time.sleep(0.2)
    bot.gripper.close()
    time.sleep(0.2)

    # Move the arm to the dropping point.
    # currently hard coded for testing

    bot.arm.set_ee_pose_components(x=x_intermidiate, y=y_intermidiate, z=z_intermidiate, roll = roll_intermidiate, pitch = pitch_intermidiate) # Moves to hard coded object drop point
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
    yolo_pub = rospy.Publisher('Center_bound', Point, queue_size=1000)
    pub_coordinates = rospy.Publisher('grasp_pose', PoseStamped, queue_size=1000) 
    pub_cloud = rospy.Publisher('cloud_stitched', CloudSamples, queue_size=10)
    arm_group = moveit_commander.MoveGroupCommander(robot_description = "/locobot/robot_description", name = "interbotix_arm", ns = "locobot")
    
    # Please uncomment if you wish to use my_ikpy and the ikpy library
    # my_chain = ikpy.chain.Chain.from_urdf_file("/home/locobot/interbotix_ws/src/interbotix_ros_rovers/interbotix_ros_xslocobots/interbotix_xslocobot_descriptions/urdf/new_locobot_urdf2.urdf")

    print("Subscribers and publishers made...")

    # Used for YOLO but camera needs to be calibrated before this is effective
    # point = yolo_find_object() # Used for yolo object detection but depth camera has not been calibrated and therefore can not be used
    # print(point)

    publish_cloud_samples() # Publishes point cloud to GPD
    print("Point cloud sent...")

    while not rospy.is_shutdown():
        x = 0
        while coordinates_done == False or grasp_info_done == False: # Waits for coordinate array and grasp score array are filled
            time.sleep(0.1)
            x = x+1
            if x > 1200: # Adds a 120 second time out for the while loop
                print("Error: Was not able to properly recieve and process information")
                break
        
        if coordinates_done == True or grasp_info_done == True:
            x = 0
            yaw_array = []
            success_array = [] # Stores whether grasp suceeds in inverse kinematics test or not
            for grasps in grasp_info_array:
                success, yaw= IK_verification(coordinate_array[x])
                success_array.append(success)
                yaw_array.append(yaw)
                if success == True:
                    rospy.signal_shutdown('Exiting ROS node')
                    break
                x = x + 1
            print("Success: " + str(success_array))
            print("Yaw: " + str(yaw_array))
        rospy.signal_shutdown('Exiting ROS node')
    rospy.spin()
       