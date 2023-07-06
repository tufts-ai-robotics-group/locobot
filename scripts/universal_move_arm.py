#!/usr/bin/env python3
# universal python script to move arms
#
# adapted from
# interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_moveit_interface/
#     scripts/moveit_python_interface
#
#

import sys
import os
import copy
from typing import Optional
import rospy
import random
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose, Point, Quaternion
from math import pi
from six.moves import input
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler
from moveit_commander.conversions import pose_to_list
from obtain_pick_item_pose import PickUpPoseCalculator

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
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_python_interface')

        ## Get the name of the robot - this will be used to properly define the end-effector link when adding a box
        print("\n".join(rospy.get_param_names()))
        self.robot_model = rospy.get_param("~robot_model")
        self.robot_name = rospy.get_namespace().strip("/")
        self.ee_link_offset = rospy.get_param("~ee_link_offset")
        self.joint_goal = rospy.get_param("~joint_goal")
        pose_goal_raw = rospy.get_param("~pose_goal")
        quat = quaternion_from_euler(pose_goal_raw[3], pose_goal_raw[4], pose_goal_raw[5])
        self.pose_goal = geometry_msgs.msg.Pose()
        self.pose_goal.position.x = pose_goal_raw[0]
        self.pose_goal.position.y = pose_goal_raw[1]
        self.pose_goal.position.z = pose_goal_raw[2]
        self.pose_goal.orientation.x = quat[0]
        self.pose_goal.orientation.y = quat[1]
        self.pose_goal.orientation.z = quat[2]
        self.pose_goal.orientation.w = quat[3]

        ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
        ## the robot:
        self.robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
        ## to the world surrounding the robot:
        self.scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to one group of joints.  In this case the group is the joints in the Interbotix
        ## arm so we set ``group_name = interbotix_arm``. If you are using a different robot,
        ## you should change this value to the name of your robot arm planning group.
        ## This interface can be used to plan and execute motions on the Interbotix Arm:
        group_name = "interbotix_" + group
        self.group = moveit_commander.MoveGroupCommander(group_name)

        ## We create a `DisplayTrajectory`_ publisher which is used later to publish
        ## trajectories for RViz to visualize:
        self.display_trajectory_publisher = rospy.Publisher("move_group/display_planned_path",
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        self.planning_frame = self.group.get_planning_frame()
        print("============ Reference frame: %s" % self.planning_frame)

        # We can also print the name of the end-effector link for this group:
        self.eef_link = self.group.get_end_effector_link()
        print("============ End effector: %s" % self.eef_link)

        # We can get a list of all the groups in the robot:
        self.group_names = self.robot.get_group_names()
        print("============ Robot Groups: " + str(self.group_names))

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(self.robot.get_current_state())
        print("")


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
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self, pose_goal: Optional[geometry_msgs.msg.Pose]=None):
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        if pose_goal is None:
            pose_goal = self.pose_goal

        print("============ Printing Pose Goal:\n" + str(pose_goal))
        self.group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        plan = self.group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.group.clear_pose_targets()

        current_pose = self.group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)


def main():
    arm_group = LBMoveIt()
    
    # go to upright
    arm_group.go_to_joint_state(LBMoveIt.ARM_JOINT_STATES.UPRIGHT)
    
    # go to some valid pose
    # arm_group.go_to_pose_goal()
    arm_group.go_to_pose_goal(Pose(
        position=Point(x=0.5, y=0, z=0.35),
        orientation=Quaternion(x=0, y=0, z=0, w=1)
    ))
    
    # go to the location of a specific object
    pose_calc = PickUpPoseCalculator("cricket_ball")
    pose = pose_calc.get_pose()
    arm_group.go_to_pose_goal(pose)

    # go to home then sleep
    arm_group.go_to_joint_state(LBMoveIt.ARM_JOINT_STATES.HOME)

    rospy.logdebug("Done!")

def test1():
    arm_group = LBMoveIt()
    arm_group.go_to_pose_goal(Pose(
        position=Point(x=0.5, y=0, z=0.2),
        orientation=Quaternion(*quaternion_from_euler(ai=0, aj=0.8, ak=0))
    ))

if __name__ == '__main__':
    main()


"""
position: 
  x: 0.5
  y: 0
  z: 0.2
orientation: 
  x: 0.0
  y: 0.3894183423086505
  z: 0.0
  w: 0.9210609940028851

"""