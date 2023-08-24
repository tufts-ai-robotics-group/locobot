#!/usr/bin/env python3

# Import required libraries and modules
import rospy
import numpy as np
from locobot_custom.srv import Facing, FacingResponse
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion

class RecycleBotGazeboFacing(object):
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('RecycleBotGazeboFacing', anonymous=True)
        
        # Create a ROS service named 'facing' that uses the Facing service type and the facing_callback function
        self.at_srv = rospy.Service('facing', Facing, self.facing_callback)
        
        # Attempt to retrieve the "facing_thresh" parameter from the ROS parameter server
        try:
            self.param_facing_thresh = rospy.get_param("facing_thresh")
        except (KeyError, rospy.ROSException):
            # Log an error and raise an exception if the parameter retrieval fails
            rospy.logerr("Error getting parameters.")
            raise ValueError
        
        # Define a mapping between model names and their corresponding PDDL names
        self.model_to_pddl_mapping = {
            "can_1": "coke_can_0",
            "ball_1": "ball",
            "bin_1": "bin",
            "doorway_1": "doorway_1"
        }

        # Keep the node running until it's shut down
        while not rospy.is_shutdown():
            rospy.spin()

    def facing_callback(self, req):
        # Callback function to check if the robot is facing the requested object
        obj = req.obj
        rospy.loginfo(f"Received request to check if robot is facing {obj}.")
        
        # Wait for a message from the '/gazebo/model_states' topic and store it in model_states
        model_states = rospy.wait_for_message('/gazebo/model_states', ModelStates)
    
        # Find the index of the robot -- "locobot" in the list of model names
        robot_index = model_states.name.index("locobot")
        robot_pose = model_states.pose[robot_index]
        robot_position = np.array([robot_pose.position.x, robot_pose.position.y])
        robot_orientation = np.array([robot_pose.orientation.x, robot_pose.orientation.y,
                                       robot_pose.orientation.z, robot_pose.orientation.w])

        # If the requested object is "nothing"
        if obj == "nothing":
            # Check if the robot is facing any of the objects in the mapping
            for key in self.model_to_pddl_mapping:
                if self.is_facing_object(key, robot_position, robot_orientation, model_states):
                    rospy.loginfo(f"Robot is facing: {key}")
                    return FacingResponse(False)  # Robot is facing an object, so not "nothing"
            return FacingResponse(True)  # Robot is not facing any objects, so "nothing"
        else:
            rospy.loginfo(f"Robot is facing: {obj}")
            # Check if the robot is facing the specified object
            return FacingResponse(self.is_facing_object(obj, robot_position, robot_orientation, model_states))

    def is_facing_object(self, obj_name, robot_position, robot_orientation, model_states):
        # Function to determine if the robot is facing a specific object
        if obj_name == "doorway_1":
            rospy.loginfo("Robot is facing doorway_1??")
            object_position = np.array([0, 0])
        elif obj_name in self.model_to_pddl_mapping:
            mapped_name = self.model_to_pddl_mapping[obj_name]
            if mapped_name not in model_states.name:
                rospy.logerr(f"Object {obj_name} not found in model states.")
                return False
            index = model_states.name.index(mapped_name)
            pose = model_states.pose[index]
            object_position = np.array([pose.position.x, pose.position.y])
        else:
            return False

        # Compute the direction vector from the robot to the object
        direction_to_object = object_position - robot_position
        direction_to_object /= np.linalg.norm(direction_to_object)
        
        # Convert the robot's quaternion orientation to Euler angles and extract the yaw
        _, _, yaw = euler_from_quaternion(robot_orientation)
        robot_facing_dir = np.array([np.cos(yaw), np.sin(yaw)])
        
        # Calculate the angle between the robot's facing direction and the direction to the object
        angle = np.arccos(np.dot(robot_facing_dir, direction_to_object))
        distance = np.linalg.norm(object_position - robot_position)

        rospy.loginfo(f"Angle: {angle}, Distance: {distance}")
        rospy.loginfo(f"Threshold: {self.param_facing_thresh[obj_name]}")
        print(f"Angle Condition: {angle < 0.25 or abs(angle - np.pi) < 0.25}")
        print(f"Distance Condition: {distance < self.param_facing_thresh[obj_name]}")

        # Check if the robot is facing the object based on the angle and distance conditions
        if (angle < 0.25 or abs(angle - np.pi) < 0.25) and distance < self.param_facing_thresh[obj_name]:
            rospy.loginfo(f"Robot is facing {obj_name}.")
            return True
        else:
            rospy.loginfo(f"Robot is not facing {obj_name}.")
            return False

# Execute the class when the script is run directly
if __name__ == "__main__":
    RecycleBotGazeboFacing()
