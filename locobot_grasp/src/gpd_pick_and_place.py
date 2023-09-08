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
import tf2_ros
import tf2_geometry_msgs
from grasp_utils import (
    generate_intermediate_pose,
)  # need to figure out what is wrong with code in this script
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose, Quaternion
from moveit_msgs.msg import PositionIKRequest
import moveit_commander
from moveit_msgs.srv import GetPositionIK
from interbotix_xs_modules.locobot import InterbotixLocobotXS
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from sensor_msgs.msg import PointCloud2, JointState
from gpd_ros.msg import GraspConfigList, CloudSources, CloudSamples
from visualization_msgs.msg import MarkerArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from locobot_msgs.srv import YoloObject

class GPDPickAndPlace:
    def __init__(self) -> None:
        # Variables to be set based on robot
        ## Can be tuned and changed according to the user.
        self.x_view = -0.008972  # Coordinates of camera
        self.y_view = -0.015818
        self.z_view = 0.000003

        self.topic_PointCloud = (
            "/locobot/camera/depth/color/points"  # Topic of the point cloud
        )
        self.arm_tf = "locobot/arm_base_link"
        self.locobot_tf = "locobot/camera_color_optical_frame"

        self.coordinate_array = []
        self.grasp_info_array = []
        self.coordinates_done = False
        self.grasp_info_done = False

        self.bot = InterbotixLocobotXS("locobot_wx200", arm_model="mobile_wx200")
        self.bot.camera.pan_tilt_move(0, 0.75)  # Set camera tilt angle

        # Resets arm position to ensure arm is ready to move to any position
        self.bot.arm.set_ee_pose_components(x=0.3, z=0.2)  # Y defaults to 0
        self.bot.arm.go_to_sleep_pose()

        rospy.Subscriber(
            "detect_grasps/plot_grasps",
            MarkerArray,
            self.callback_coordinates,
            queue_size=100,
        )  # Crate publishers and subscribers
        rospy.Subscriber(
            "detect_grasps/clustered_grasps",
            GraspConfigList,
            self.callback_grasp_info,
            queue_size=100,
        )
        self.pub_coordinates = rospy.Publisher(
            "grasp_pose", PoseStamped, queue_size=1000
        )
        self.pub_cloud = rospy.Publisher("cloud_stitched", CloudSamples, queue_size=10)

        self.yolo_find_objects = rospy.ServiceProxy("yolo_find_object", YoloObject)

        self.arm_group = moveit_commander.MoveGroupCommander(
            robot_description="/locobot/robot_description",
            name="interbotix_arm",
            ns="locobot",
        )

        # Please uncomment if you wish to use my_ikpy and the ikpy library
        # my_chain = ikpy.chain.Chain.from_urdf_file("/home/locobot/interbotix_ws/src/interbotix_ros_rovers/interbotix_ros_xslocobots/interbotix_xslocobot_descriptions/urdf/new_locobot_urdf2.urdf")

        # Used for YOLO but camera needs to be calibrated before this is effective
        # point = yolo_find_object() # Used for yolo object detection but depth camera has not been calibrated and therefore can not be used
        # print(point)

        self.publish_cloud_samples()  # Publishes point cloud to GPD
        print("Point cloud sent...")

        while not rospy.is_shutdown():
            yolo_objects = input("Enter a list of objects to find seperated by comma's: ")
            yolo_objects = yolo_objects.split(",")
            yolo_objects = [x.strip() for x in yolo_objects]

            try:
                self.yolo_find_objects(yolo_objects)
            except rospy.ServiceException as error:
                rospy.logerr(error)
                continue
            except ValueError as error:
                rospy.logerr(error)
                continue

            x = 0
            while self.coordinates_done == False or self.grasp_info_done == False:
                time.sleep(0.1)
                x = x + 1
                if x > 1200:  # Adds a 120 second time out for the while loop
                    print(
                        "Error: Was not able to properly recieve and process information"
                    )
                    break

            if self.coordinates_done == True or self.grasp_info_done == True:
                x = 0
                yaw_array = []
                success_array = (
                    []
                )  # Stores whether grasp suceeds in inverse kinematics test or not
                for grasps in self.grasp_info_array:
                    success, yaw = self.IK_verification(self.coordinate_array[x])
                    success_array.append(success)
                    yaw_array.append(yaw)
                    if success == True:
                        rospy.signal_shutdown("Exiting ROS node")
                        break
                    x = x + 1
                rospy.loginfo("Success: " + str(success_array))
                rospy.loginfo("Yaw: " + str(yaw_array))
            rospy.signal_shutdown("Exiting ROS node")
        rospy.spin()

    def publish_cloud_samples(self):
        # Creates cloud samples message that is fed into GPD

        cloud_samples_msg = CloudSamples()  # Create a new CloudSamples message
        cloud_sources_msg = self.create_cloud_sources()
        cloud_samples_msg.cloud_sources = (
            cloud_sources_msg  # Stores cloud sources message in cloud samples message
        )

        self.pub_cloud.publish(cloud_samples_msg)
        rospy.sleep(3)  # workaround for GPD buffer
        self.pub_cloud.publish(
            cloud_samples_msg
        )  # This publishes the message a second time as we had issues only sending one message or sending 2 rapidly

    def create_cloud_sources(
        self,
    ):  # Creates cloud sources message that is stored in cloud samples message
        cloud_sources_msg = CloudSources()
        cloud_sources_msg.cloud = rospy.wait_for_message(
            self.topic_PointCloud, PointCloud2, timeout=10
        )  # Gets point cloud from camera

        cloud_sources_msg.view_points = [
            Point(
                np.float64(self.x_view),
                np.float64(self.y_view),
                np.float64(self.z_view),
            )
        ]  # Coordinates of camera
        return cloud_sources_msg

    def callback_coordinates(self, msg):
        # Stores grasp coordinate information from GPD
        pose = Pose()

        i = 0
        while i < (
            len(msg.markers) / 4
        ):  # Message has 4 markers for each part of end effector. We only need to look at oe part per pose
            pose = PoseStamped()
            marker = msg.markers[
                3 + (i * 4)
            ]  # Iterate through grasp info and only look at last part which is the base of end effector
            quaternion = (  # Extract the quaternion components from the marker message
                marker.pose.orientation.x,
                marker.pose.orientation.y,
                marker.pose.orientation.z,
                marker.pose.orientation.w,
            )
            pose.pose.orientation = Quaternion(*quaternion)
            pose.pose.position = marker.pose.position
            self.coordinate_array.append(pose)
            i = i + 1

        self.coordinates_done = True

    def callback_grasp_info(self, msg):
        # Stores grasp score in an array

        i = 0
        while i < len(msg.grasps):
            score = msg.grasps[i].score
            self.grasp_info_array.append(score)
            i = i + 1

        self.grasp_info_done = True

    def my_ikpy(self, pose):
        # IK function using the python ikpy library which should be able to
        # parse through data much more quickly
        x_pos, y_pos, z_pos = (
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
        )
        angle = euler_from_quaternion([
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ])

        roll=angle[0]
        pitch=angle[1]
        yaw=angle[2]

        if (
            -0.524 < yaw < 0.524
        ):  # Filtering to ensure yaw is within range graspable to arm
            yaw = math.atan2(
                pose.pose.position.y, pose.pose.position.x
            )  # Manually calculates yaw as nature of 5 degree arm makes it hard to control

        target_position = [x_pos, y_pos, z_pos]
        target_orientation = [roll, pitch, pitch]

        # Perform inverse kinematics
        target_pose = np.concatenate((target_position, target_orientation))
        joint_angles = self.my_chain.inverse_kinematics(target_pose)
        reachable = self.my_chain.is_possible(
            joint_angles
        )  # Returns boolean on if grasp is possible
        return reachable

    def solve_ik(self, pose):
        # IK function using compute ik topic
        # ik topic can take in coordinates and perform inverse kinematics to figure out of grasp is reachable
        request = PositionIKRequest()

        joint_state_locobot = rospy.wait_for_message(
            "/locobot/joint_states", JointState, timeout=10
        )

        request.group_name = "interbotix_arm"
        request.robot_state.joint_state = joint_state_locobot
        request.pose_stamped = pose

        request.timeout = rospy.Duration(5, 0)

        try:
            # Call the IK service
            rospy.wait_for_service("/locobot/compute_ik")
            compute_ik_service = rospy.ServiceProxy(
                "/locobot/compute_ik", GetPositionIK
            )
            response = compute_ik_service(request)

            if response.error_code.val == response.error_code.SUCCESS:
                # IK solution found
                joint_positions = response.solution.joint_state.position
                return True, joint_positions
            else:
                # IK solution not found
                return False, []

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed:", e)
            return False, []

    def IK_verification(self, pose):
        # Inverse kinematics function to esure grasp is reachable by arm
        # computing the orientations
        angle = euler_from_quaternion([
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ])
        roll=angle[0]
        pitch=angle[1]
        yaw=angle[2]

        if (
            1.5708 < yaw
        ):  # GPD assumes what is behind object because there is only 1 view so we find inverse of yaw if it is behind object
            yaw = 3.13159 - yaw  # get the inverse of the yaw
        elif -1.5708 > yaw:  # fropm the other side
            yaw = 3.13159 + yaw
        if (
            -0.17 < yaw < 0.17
        ):  # This restricts the yaw to what 10 degrees since yaw is determiuned by x,y position of end effector
            yaw = math.atan2(
                pose.pose.position.y, pose.pose.position.x
            )  # Manually calculates yaw as nature of 5 degree arm makes it hard to control
            quat = quaternion_from_euler(roll, pitch, yaw)

            pose.pose.orientation.x = quat[0] 
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]  # sorry for the crude way

            self.arm_group.set_pose_target(
                pose, end_effector_link="locobot/gripper_link"
            )  # Set target pose
            (
                error_code,
                _,
                planning_time,
                _,
            ) = (
                self.arm_group.plan()
            )  # Plan motion and return whether point is viable or not/calculation time
            print(
                "Plan success: "
                + str(error_code)
                + " Planning time: "
                + str(planning_time)
            )

            # Performs transfromaton from one link to other
            buffer = tf2_ros.Buffer()
            tf2_ros.TransformListener(
                buffer
            )  # Listens to tf to get transformaton information necessary
            rospy.sleep(2)  # Provides time for buffer to collect data
            target_transform: TransformStamped = buffer.lookup_transform(
                self.arm_tf, self.locobot_tf, rospy.Time(0)
            )
            target_pose = tf2_geometry_msgs.do_transform_pose(pose, target_transform)

            if error_code == True:
                print(target_pose)
                self.move_arm(target_pose)

            return error_code, yaw

            # Set other planning parameters if needed
        else:
            error_code = False
            return error_code, yaw

    def move_arm(self, pose):  # Function to pick up and place object
        self.pub_coordinates.publish(pose)

        x, y, z = pose.pose.position.x, pose.pose.position.y, pose.pose.position.z
        angle = euler_from_quaternion([
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ])

        roll=angle[0]
        pitch=angle[1]
        yaw=angle[2]

        yaw = math.atan2(pose.pose.position.y, pose.pose.position.x)

        # Creates aproach position as to not knock object over
        roll = 0  # set to 0 to constrain GPD until we figure out how to filter out incorrect rolls
        object_pose = np.array(
            [x, y, z, roll, pitch, yaw]
        )  # replace with your actual values
        delta = -0.1  # offset
        axis = 0  # 0 for x-axis, 1 for y-axis, 2 for z-axis
        (
            x_intermidiate,
            y_intermidiate,
            z_intermidiate,
            roll_intermidiate,
            pitch_intermidiate,
            yaw_intermidiate,
        ) = generate_intermediate_pose(object_pose, delta, axis)
        delta_final = 0.01
        # this adjusts the final grasp- poise to get a bit deeper into the object to succesfully grasp it
        (
            x_final,
            y_final,
            z_final,
            roll_final,
            pitch_final,
            yaw_final,
        ) = generate_intermediate_pose(object_pose, delta_final, axis)

        print("Moving arm...")
        print(
            "x = {} y = {} z = {} roll = {} pitch = {} yaw = {}".format(
                x_intermidiate,
                y_intermidiate,
                z_intermidiate,
                roll_intermidiate,
                pitch_intermidiate,
                yaw_intermidiate,
            )
        )
        self.bot.arm.set_ee_pose_components(x=0.3, z=0.2)
        self.bot.arm.set_ee_pose_components(
            x=x_intermidiate,
            y=y_intermidiate,
            z=z_intermidiate,
            roll=roll_intermidiate,
            pitch=pitch_intermidiate,
        )  # Moves slightly behind object as to not contact it
        time.sleep(0.2)
        self.bot.arm.set_ee_pose_components(
            x=x_final, y=y_final, z=z_final, roll=roll_final, pitch=pitch_final
        )  # Move to object to pick up
        time.sleep(0.2)
        self.bot.gripper.close()
        time.sleep(0.2)

        # Move the arm to the dropping point.
        # currently hard coded for testing

        self.bot.arm.set_ee_pose_components(
            x=x_intermidiate,
            y=y_intermidiate,
            z=z_intermidiate,
            roll=roll_intermidiate,
            pitch=pitch_intermidiate,
        )  # Moves to hard coded object drop point
        time.sleep(0.2)
        self.bot.arm.set_ee_pose_components(x=0.4, y=-0.3, z=0.2, moving_time=1.5)
        time.sleep(0.2)
        self.bot.gripper.open()
        time.sleep(0.2)

        self.bot.arm.set_ee_pose_components(
            x=0.3, z=0.2, moving_time=1.5
        )  # Moves arm to sleep position
        self.bot.arm.go_to_sleep_pose()
        time.sleep(0.1)


if __name__ == "__main__":
    GPDPickAndPlace()
