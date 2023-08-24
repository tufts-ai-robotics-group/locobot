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

        self.model_to_pddl_mapping = {
            "robot_1": "locobot",
            "can_1": "coke_can_0",
            "ball_1": "ball",
            "bin_1": "bin",
        }

        while not rospy.is_shutdown():
            rospy.spin()

    # def at_callback(self, req):
    #     model_states = rospy.wait_for_message('/gazebo/model_states', ModelStates)
        
    #     room = req.room
    #     obj = req.obj
    #     rospy.loginfo(f"Received request to check if {obj} is in {room}.")

    #     if obj not in self.model_to_pddl_mapping:
    #         return AtResponse(False)

    #     obj_mapped = self.model_to_pddl_mapping[obj]

    #     if obj_mapped not in model_states.name:
    #         rospy.logerr(f"Object {obj_mapped} not found in model states.")
    #         return AtResponse(False)

    #     index = model_states.name.index(obj_mapped)
    #     pose = model_states.pose[index]
    #     rospy.loginfo(f"Pose of {obj} is {pose}.")
    #     object_position = np.array([pose.position.x, pose.position.y])
    #     rospy.loginfo(f"Object {obj} is at {object_position}.")

    #     at = "nothing"
    #     point = (object_position[0], object_position[1])
    #     print(point)
    #     for boundary_name, boundary_polygon in self.param_at_boundaries.items():

    #         boundary = [(x, y) for [x, y] in boundary_polygon]

    #         if self.is_point_inside_polygon(point, boundary):
    #             at = boundary_name
    #             rospy.loginfo(f"Object {obj} determined to be in {at}.") # Adding this for clarity
    #             if boundary_name == "nothing":
    #                 rospy.loginfo(f"Object {obj} is not in {room}.")
    #                 break
        
    #     print (at)

    #     if at == room:
    #         rospy.loginfo(f"Object {obj} is in {room}.") # Logging confirmation
    #         return AtResponse(True)
    #     else:
    #         rospy.loginfo(f"Object {obj} is NOT in {room}. It's in {at}.") # More detailed error message
    #         return AtResponse(False)
        
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
        rospy.loginfo(f"Pose of {obj} is {pose}.")
        object_position = np.array([pose.position.x, pose.position.y])
        rospy.loginfo(f"Object {obj} is at {object_position}.")

        point = (object_position[0], object_position[1])

        # Check if point is in requested room
        boundary = self.param_at_boundaries[room]
        if self.is_point_inside_polygon(point, self.param_at_boundaries['room_1']):
            rospy.loginfo(f"Point {point} is inside room_1 boundaries.")
        else:
            rospy.loginfo(f"Point {point} is NOT inside room_1 boundaries.")

        if self.is_point_inside_polygon(point, boundary):
            rospy.loginfo(f"Object {obj} is in {room}.")
            return AtResponse(True)
        else:
            rospy.loginfo(f"Object {obj} is NOT in {room}.")
            return AtResponse(False)



        
    def is_point_inside_polygon(self, point_coords, boundary_coords):
        """
        Check if a given point is inside a polygon defined by boundary coordinates.
        :param point_coords: Tuple of (x, y) coordinates for the point.
        :param boundary_coords: List of tuples, each tuple being (x, y) coordinates of a vertex of the polygon.
        :return: True if the point is inside the polygon, False otherwise.
        """
        point = Point(point_coords)
        poly = Polygon(boundary_coords)
        return point.within(poly)


    # def is_point_inside_polygon(self, point, polygon):
    #     """
    #     Determines if the point is inside the polygon.

    #     Args:
    #     - point (tuple): A tuple representing the point (x, y).
    #     - polygon (list): A list of tuples representing the vertices of the polygon in sequential order.

    #     Returns:
    #     - bool: True if the point is inside the polygon, False otherwise.
    #     """
    #     x, y = point
    #     num_vertices = len(polygon)
    #     odd_nodes = False
    #     j = num_vertices - 1  # The last vertex is the previous one to the first vertex

    #     for i in range(num_vertices):
    #         xi, yi = polygon[i]
    #         xj, yj = polygon[j]
    #         if yi < y and yj >= y or yj < y and yi >= y:
    #             if xi + (y - yi) / (yj - yi) * (xj - xi) < x:
    #                 odd_nodes = not odd_nodes
    #         j = i

        return odd_nodes

        
if __name__ == "__main__":
    RecycleBotGazeboAt()