#!/usr/bin/env python3

# import rospy
# from geometry_msgs.msg import PoseStamped

import math
import sys
import os
import copy
from typing import Optional
import rospy
import random
import time
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from math import pi
from six.moves import input
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler
from moveit_commander.conversions import pose_to_list
from locobot_custom.srv import Grasp, GraspResponse
from locobot_custom.srv import GraspPose, GraspPoseResponse

def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal         A list of floats, a Pose or a PoseStamped
    @param: actual       A list of floats, a Pose or a PoseStamped
    @param: tolerance    A float
    @returns: bool
    """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True

class LBMoveIt:
    class ARM_JOINT_STATES:
        HOME = [0, 0, 0, 0, 0]
        SLEEP = [0, -1.29154, 1.55334, 0.698132, 0]
        UPRIGHT = [0, 0, -1.5708, 0, 0]


    def __init__(self, group="arm"): # group: arm / gripper
        # moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node('moveit_python_interface')

        ## Get the name of the robot - this will be used to properly define the end-effector link when adding a box
        # print("\n".join(rospy.get_param_names()))
        # self.robot_model = rospy.get_param("~robot_model")
        # self.robot_name = rospy.get_namespace().strip("/")
        # self.ee_link_offset = rospy.get_param("~ee_link_offset")
        # self.joint_goal = rospy.get_param("~joint_goal")
        # pose_goal_raw = rospy.get_param("~pose_goal")
        # quat = quaternion_from_euler(pose_goal_raw[3], pose_goal_raw[4], pose_goal_raw[5])
        # self.pose_goal = geometry_msgs.msg.Pose()
        # self.pose_goal.position.x = pose_goal_raw[0]
        # self.pose_goal.position.y = pose_goal_raw[1]
        # self.pose_goal.position.z = pose_goal_raw[2]
        # self.pose_goal.orientation.x = quat[0]
        # self.pose_goal.orientation.y = quat[1]
        # self.pose_goal.orientation.z = quat[2]
        # self.pose_goal.orientation.w = quat[3]

        ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
        ## the robot:
        self.robot = moveit_commander.RobotCommander(robot_description="/locobot/robot_description", ns="locobot")

        ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
        ## to the world surrounding the robot:
        self.scene = moveit_commander.PlanningSceneInterface(ns="locobot")

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to one group of joints.  In this case the group is the joints in the Interbotix
        ## arm so we set ``group_name = interbotix_arm``. If you are using a different robot,
        ## you should change this value to the name of your robot arm planning group.
        ## This interface can be used to plan and execute motions on the Interbotix Arm:
        group_name = "interbotix_" + group
        self.group = moveit_commander.MoveGroupCommander(robot_description="/locobot/robot_description", name = group_name, ns="locobot")

        ## We create a `DisplayTrajectory`_ publisher which is used later to publish
        ## trajectories for RViz to visualize:test1ue_size=20)

        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        self.planning_frame = self.group.get_planning_frame()
        print("============ Reference frame: %s" % self.planning_frame)

        # We can also print the name of the end-effector link for this group:
        self.eef_link = self.group.get_end_effector_link()
        # self.group.get_end_effector_link()
        # self.eef_link = "gripper_link"
        # print("self.group._has_end_effector_link(): " + str(self.group._has_end_effector_link()))
        print("============ End effector: %s" % self.eef_link)

        # We can get a list of all the groups in the robot:
        self.group_names = self.robot.get_group_names()
        print("============ Robot Groups: " + str(self.group_names))

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:

        print("============ Printing robot state")

        print(self.robot.get_current_state())
        print("LBMoveit initialized")
        # subscribe to the estimated grasp pose topic
        self.grasp_pose = None

    def go_plan(self, pose: PoseStamped):
        print ("Current Joint Values in func go_plan: " + str(self.group.get_current_joint_values()))
        print ("pose: " + str(pose))
        self.group.set_pose_target(pose, end_effector_link="locobot/gripper_link") # Set target pose
        error_code, _, planning_time , _ = self.group.plan()
        print("Planning time: " + str(planning_time))
        print("Error code: " + str(error_code))
        return error_code

    def go_to_joint_state(self, joint_goal=None):
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        if joint_goal is None:
            joint_goal = self.joint_goal
        
        print("============ Printing Joint Goal: " + str(joint_goal))

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        self.group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.group.stop()

        current_joints = self.group.get_current_joint_values()
        print ("Current Joint Values in func go_to_joint_state: " + str(current_joints))
        tolerance = 0.01
        return all_close(joint_goal, current_joints, tolerance)

    def go_to_pose_goal(self, pose_goal: Optional[geometry_msgs.msg.Pose]=None):
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        if pose_goal is None:
            pose_goal = self.pose_goal
        current_joint_values = self.group.get_current_joint_values()
        print ("Current Joint Values in func go_to_pose_goal: " + str(current_joint_values))
        current_pose = self.group.get_current_pose().pose
        print ("Current Pose in func go_to_pose_goal : " + str(current_pose))

        print("============ Printing Pose Goal:\n" + str(pose_goal))
        xyz = [pose_goal.position.x, pose_goal.position.y, pose_goal.position.z]
        print (self.group.get_goal_tolerance())
        # self.group.set_goal_position_tolerance(value=0.01)
        self.group.set_goal_tolerance(value=0.1)
        # self.group.set_goal_orientation_tolerance(value=0.1)
        print (self.group.get_goal_tolerance())
        self.group.set_position_target(xyz) # this works
        # self.group.set_pose_target(pose_goal) # this doesnt work
        print("!!!!!!!!!!! Planning NOW !!!!!")

        ## Now, we call the planner to compute the plan and execute it.
        start_time = time.time()
        duration = 10 # Specify the duration in seconds

        while time.time() - start_time < duration:
            try:
                plan = self.group.go(wait=True)
                if plan:
                    break  # Exit the loop since the condition is met
            except Exception as e:
                # Handle the exception, if needed
                print("An error occurred:", e)

        print("plan = {}".format(plan))
        # Calling `stop()` ensures that there is no residual movement
        self.group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.group.clear_pose_targets()

        current_pose = self.group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)
    
def handle_grasp(req):
    """
    Handle the grasp request, listen to the appropriate topic for the pose, 
    move the arm to the received pose, and close the gripper.
    
    Args:
    - req: The request containing the target name.
    
    Returns:
    - dict: A dictionary indicating success status and an associated message.
    """
    # Listen to the topic to get the pose
    topic_name = "/computed_grasp_pose/" + req.target.data
    pose_msg = rospy.wait_for_message(topic_name, PoseStamped, timeout=10)  # Adjust timeout as needed
    # rospy.loginfo("===============================>>>>>>>>>>>>>>>>>>>>>>")
    # rospy.loginfo("__________________________________________________________")
    # rospy.loginfo("Received pose message from topic: {}".format(topic_name))
    # rospy.loginfo("Pose message: {}".format(pose_msg))
    if not pose_msg:
        return GraspResponse(success=False, message=String(data="Failed to get grasp pose from topic"))
    # Initialize the arm and gripper group
    arm_group = LBMoveIt(group="arm")
    gripper_group = LBMoveIt(group="gripper")
    
    # Attempt to move the arm to the desired pose
    if not arm_group.go_to_pose_goal(pose_msg.pose):
        return GraspResponse(success=False, message=String(data="Failed to move arm to target pose"))
    # Grasp the object
    gripper_group.close_gripper()

    return GraspResponse(success=True, message=String(data="Successfully grasped object at target pose"))

def grasp_server():
    """
    Initialize the grasp service and keep it running.
    """
    rospy.init_node('grasp_service')
    s = rospy.Service('grasp', Grasp, handle_grasp)
    rospy.spin()

if __name__ == "__main__":
    grasp_server()
