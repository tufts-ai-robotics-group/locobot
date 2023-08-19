import rospy
import numpy as np
from shapely.geometry import Point
from shapely.geometry import Polygon

from locobot_custom.srv import At, AtResponse
from gazebo_msgs.msg import ModelStates

class RecycleBotGazeboAt(object):

    def __init__(self):

        rospy.init_node('RecycleBotGazeboAt', anonymous=True)

        self.at_srv = rospy.Service('at', At, self.at_callback)

        try:
            self.param_at_boundaries = rospy.get_param("at_boundaries")
        except (KeyError, rospy.ROSException):
            rospy.logerr("Error getting parameters.")
            raise ValueError

        self.at_boundaries = {}
        for boundary_name, boundary_edges in self.param_at_boundaries.items():
            edges = []
            for edge in boundary_edges:
                edges.append((edge[0], edge[1]))
            polygon = Polygon(edges)
            self.at_boundaries[boundary_name] = polygon


        while not rospy.is_shutdown():
            rospy.spin()

    def at_callback(self, room, obj):
        model_states = rospy.wait_for_message('/gazebo/model_states', ModelStates)

        index = model_states.name.index(obj)
        pose = model_states.pose[index]
        position = np.array([pose.position.x, pose.position.y]) 

        at = "nothing"
        point = Point(position[0], position[1])
        for boundary_name, boundary_polygon in self.at_boundaries.items():
            if boundary_polygon.contains(point):
                at = boundary_name
                if boundary_name == "nothing":
                    break

        if at == room:
            return AtResponse(True)