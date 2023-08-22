#!/usr/bin/env python3
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
        rospy.loginfo(f"at_boundaries: {self.at_boundaries}")

        self.model_to_pddl_mapping = {
            "robot_1": "locobot",
            "can_1": "coke_can_0",
            "ball_1": "cricket_ball",
            "bin_1": "bin",
        }

        while not rospy.is_shutdown():
            rospy.spin()

    def at_callback(self, req):
        model_states = rospy.wait_for_message('/gazebo/model_states', ModelStates)
        
        room = req.room
        obj = req.obj
        rospy.loginfo(f"Received request to check if {obj} is in {room}.")

        if obj not in self.model_to_pddl_mapping:
            return AtResponse(False)

        obj_mapped = self.model_to_pddl_mapping[obj]

        if obj_mapped not in model_states.name:
            rospy.logerr(f"Object {obj_mapped} not found in model states.")
            return AtResponse(False)

        index = model_states.name.index(obj_mapped)
        pose = model_states.pose[index]
        object_position = np.array([pose.position.x, pose.position.y])
        rospy.loginfo(f"Object {obj} is at {object_position}.")

        at = "nothing"
        point = Point(object_position[0], object_position[1])
        for boundary_name, boundary_polygon in self.at_boundaries.items():
            if boundary_polygon.contains(point):
                at = boundary_name
                rospy.loginfo(f"Object {obj} determined to be in {at}.") # Adding this for clarity
                if boundary_name == "nothing":
                    rospy.loginfo(f"Object {obj} is not in {room}.")
                    break

        if at == room:
            rospy.loginfo(f"Object {obj} is in {room}.") # Logging confirmation
            return AtResponse(True)
        else:
            rospy.loginfo(f"Object {obj} is NOT in {room}. It's in {at}.") # More detailed error message
            return AtResponse(False)

        
if __name__ == "__main__":
    RecycleBotGazeboAt()