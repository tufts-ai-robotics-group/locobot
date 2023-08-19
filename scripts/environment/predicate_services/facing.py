#!/usr/bin/env python3

import rospy
import numpy as np

from locobot_custom.srv import Facing, FacingResponse
from gazebo_msgs.msg import ModelStates

class RecycleBotGazeboFacing(object):

    def __init__(self):

        rospy.init_node('RecycleBotGazeboFacing', anonymous=True)

        self.at_srv = rospy.Service('facing', Facing, self.facing_callback)

        try:
            self.param_facing_thresh = rospy.get_param("facing_thresh")
        except (KeyError, rospy.ROSException):
            rospy.logerr("Error getting parameters.")
            raise ValueError
        
        self.model_to_pddl_mapping = {
            "can_1": "coke_can_0",
            "ball_1": "cricket_ball",
            "bin_1": "bin",
            "doorway_1": "doorway_1"
        }

        while not rospy.is_shutdown():
            rospy.spin()

    def facing_callback(self, req):
        obj = req.obj

        position = []

        model_states = rospy.wait_for_message('/gazebo/model_states', ModelStates)

        robot_index = model_states.name.index("locobot")
        robot_pose = model_states.pose[robot_index]
        robot_position = np.array([robot_pose.position.x, robot_pose.position.y])
        robot_orientation = np.array([robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w])

        if obj == "nothing":
            for key, _ in self.model_to_pddl_mapping.items():
                if self.facing_callback(key):
                    return FacingResponse(False)
            return FacingResponse(True)

        elif obj == "doorway_1":
            position = np.array([0, 0])
        elif obj in self.model_to_pddl_mapping:
            obj = self.model_to_pddl_mapping[obj]
            index = model_states.name.index(obj)
            pose = model_states.pose[index]
            position = np.array([pose.position.x, pose.position.y])
        else:
            return FacingResponse(False)
        
        # Get the angle between the robot and the object if the angle is less than 0.25 radians and 
        # the object is within the param_facing_thresh distance of the robot, then set facing to true

        facing = False 
        if np.linalg.norm(position - robot_position) < self.param_facing_thresh[req.obj]:
            robot_orientation = robot_orientation / np.linalg.norm(robot_orientation)
            object_orientation = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
            object_orientation = object_orientation / np.linalg.norm(object_orientation)
            angle = np.arccos(np.dot(robot_orientation, object_orientation))
            if angle < 0.25:
                facing = True

        return FacingResponse(facing)

if __name__ == "__main__":
    RecycleBotGazeboFacing()