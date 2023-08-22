#!/usr/bin/env python3
import rospy
import numpy as np

from locobot_custom.srv import Hold, HoldResponse
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import JointState
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf

class RecycleBotGazeboHold(object):

    def __init__(self):

        rospy.init_node('RecycleBotGazeboHold', anonymous=True)

        self.at_srv = rospy.Service('hold', Hold, self.hold_callback)

        try:
            self.hold_thresholds = rospy.get_param("hold_thresh")
        except (KeyError, rospy.ROSException):
            rospy.logerr("Error getting parameters.")
            raise ValueError
        
        self.model_to_pddl_mapping = {
            "can_1": "coke_can_0",
            "ball_1": "cricket_ball_smaller",
        }

        while not rospy.is_shutdown():
            rospy.spin()

    def hold_callback(self, req):
        result = self.process_obj(req.obj)
        return HoldResponse(result)

    def process_obj(self, obj_name):
        rospy.loginfo(f"Received request to check if robot is holding {obj_name}.")
        if obj_name not in self.model_to_pddl_mapping:
            rospy.logwarn(f"Object {obj_name} not defined in the mapping.")
            return False

        mapped_obj_name = self.model_to_pddl_mapping[obj_name]
        model_states = rospy.wait_for_message('/gazebo/model_states', ModelStates)

        listener = tf.TransformListener()
        listener.waitForTransform("/locobot/odom", "/locobot/gripper_link", rospy.Time(), rospy.Duration(4.0))
        (trans, rot) = listener.lookupTransform("/locobot/odom", "/locobot/gripper_link", rospy.Time(0))
        euler = euler_from_quaternion(rot)

        if mapped_obj_name not in model_states.name:
            rospy.logwarn(f"{mapped_obj_name} not found in gazebo model states.")
            return False

        index = model_states.name.index(mapped_obj_name)
        pose = model_states.pose[index]
        position = np.array([pose.position.x, pose.position.y, pose.position.z]) 

        hold = "nothing"
        for holdable, threshold in self.hold_thresholds.items():
            if np.linalg.norm(position - trans) < threshold:
                hold = holdable

        return hold == mapped_obj_name
    
if __name__ == "__main__":
    RecycleBotGazeboHold()
