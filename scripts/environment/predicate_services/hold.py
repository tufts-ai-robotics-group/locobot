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
            "ball_1": "cricket_ball",
        }

        while not rospy.is_shutdown():
            rospy.spin()

    def hold_callback(self, req):

        print(req)
        print(req.obj)
        obj = req.obj

        if obj in self.model_to_pddl_mapping:
            obj = self.model_to_pddl_mapping[obj]
        elif obj != "nothing":
            return HoldResponse(False)

        model_states = rospy.wait_for_message('/gazebo/model_states', ModelStates)
        # Write ros code to get the transform of the locobot/odom frame to the locobot/gripper_link frame
        listener = tf.TransformListener()
        listener.waitForTransform("/locobot/odom", "/locobot/gripper_link", rospy.Time(), rospy.Duration(4.0))
        (trans, rot) = listener.lookupTransform("/locobot/odom", "/locobot/gripper_link", rospy.Time(0))
        euler = euler_from_quaternion(rot)

        if obj != "nothing":
            index = model_states.name.index(obj)
            pose = model_states.pose[index]
            position = np.array([pose.position.x, pose.position.y, pose.position.z]) 

            hold = "nothing"
            for holdable, threshold in self.hold_thresholds.items():
                if np.linalg.norm(position - trans) < threshold:
                    hold = holdable
        else:
            for key, _ in self.model_to_pddl_mapping.items():
                if self.hold_callback(key):
                    return HoldResponse(False)

        return HoldResponse(hold == obj)
    
if __name__ == "__main__":
    RecycleBotGazeboHold()