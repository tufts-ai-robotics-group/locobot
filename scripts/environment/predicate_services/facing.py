import rospy
import numpy as np

from locobot_custom.srv import Facing, FacingResponse
from gazebo_msgs.msg import ModelStates

class RecycleBotGazeboAt(object):

    def __init__(self):

        rospy.init_node('RecycleBotGazeboFacing', anonymous=True)

        self.at_srv = rospy.Service('facing', Facing, self.facing_callback)

        try:
            self.param_facing_thresh = rospy.get_param("facing_thresh")
        except (KeyError, rospy.ROSException):
            rospy.logerr("Error getting parameters.")
            raise ValueError

        while not rospy.is_shutdown():
            rospy.spin()

    def facing_callback(self, room, obj):
        model_states = rospy.wait_for_message('/gazebo/model_states', ModelStates)

        robot_index = model_states.name.index("robot")
        robot_pose = model_states.pose[robot_index]
        robot_position = np.array([robot_pose.position.x, robot_pose.position.y])
        robot_orientation = np.array([robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w])

        index = model_states.name.index(obj)
        pose = model_states.pose[index]
        position = np.array([pose.position.x, pose.position.y])
     
        # Get the angle between the robot and the object if the angle is less than 0.25 radians and 
        # the object is within the param_facing_thresh distance of the robot, then set facing to true

        facing = False 
        if np.linalg.norm(position - robot_position) < self.param_facing_thresh:
            robot_orientation = robot_orientation / np.linalg.norm(robot_orientation)
            object_orientation = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
            object_orientation = object_orientation / np.linalg.norm(object_orientation)
            angle = np.arccos(np.dot(robot_orientation, object_orientation))
            if angle < 0.25:
                facing = True

        return FacingResponse(facing)

       