import rospy
import numpy as np
from shapely.geometry import Point
from shapely.geometry import Polygon

from locobot_custom.srv import Contain, ContainResponse
from gazebo_msgs.msg import ModelStates

class RecycleBotGazeboContain(object):

    def __init__(self):

        rospy.init_node('RecycleBotGazeboAt', anonymous=True)

        self.contain_srv = rospy.Service('contain', Contain, self.at_callback)

        while not rospy.is_shutdown():
            rospy.spin()

    def contain_callback(self, obj, container):
        if container != "bin_1":
            return ContainResponse(False)
        
        model_states = rospy.wait_for_message('/gazebo/model_states', ModelStates)
        
        index = model_states.name.index(obj)
        pose = model_states.pose[index]
        position = np.array([pose.position.x, pose.position.y])

        bin_pos = np.array([4.87, 2.07])
        bin_rad = 0.1 

        return ContainResponse(np.linalg.norm(position - bin_pos) < bin_rad)
            
    