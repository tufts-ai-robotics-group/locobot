import rospy
import numpy as np

from locobot_custom.srv import Hold, HoldResponse
from gazebo_msgs.msg import ModelStates


class RecycleBotGazeboHold(object):

    def __init__(self):

        rospy.init_node('RecycleBotGazeboHold', anonymous=True)

        self.at_srv = rospy.Service('hold', Hold, self.hold_callback)

        try:
            self.hold_thresholds = rospy.get_param("hold_thresholds")
        except (KeyError, rospy.ROSException):
            rospy.logerr("Error getting parameters.")
            raise ValueError

        while not rospy.is_shutdown():
            rospy.spin()

    def hold_callback(self, obj):
        model_states = rospy.wait_for_message('/gazebo/model_states', ModelStates)

        index = model_states.name.index(obj)
        pose = model_states.pose[index]
        position = np.array([pose.position.x, pose.position.y]) 

        hold = "nothing"
        for holdable, threshold in self.hold_thresholds.items():
            if holdable == "nothing":
                continue
            if np.linalg.norm(position - np.array(threshold)) < 0.1:
                hold = holdable

        return HoldResponse(hold == obj)