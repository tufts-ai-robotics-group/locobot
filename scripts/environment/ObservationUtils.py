#!/usr/bin/env python3
import rospy
import numpy as np
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion

class GazeboObservationGenerator:
    def __init__(self, robot_name):
        # Initialize ROS node
        rospy.init_node('gazebo_observation_generator')

        # Store the name of the robot
        self.robot_name = robot_name

    def get_occupancy_grid(self):
        # Get model states
        model_states = rospy.wait_for_message('/gazebo/model_states', ModelStates)

        # Find the min and max x and y coordinates of the walls
        min_x = min_y = float('inf')
        max_x = max_y = float('-inf')
        for name, pose in zip(model_states.name, model_states.pose):
            if 'wall' in name:  
                x = pose.position.x
                y = pose.position.y
                min_x = min(min_x, x)
                min_y = min(min_y, y)
                max_x = max(max_x, x)
                max_y = max(max_y, y)

        # Calculate the size of the environment
        width = max_x - min_x + 1  # add 1 to ensure the maximum x coordinate fits within the grid
        height = max_y - min_y + 1  # add 1 to ensure the maximum y coordinate fits within the grid

        # Create an empty grid
        grid = np.zeros((int(height), int(width)))

        # Mark the positions of the objects in the grid
        for name, pose in zip(model_states.name, model_states.pose):
            if 'wall' in name: # Only consider walls for the grid
                # Get the length of the wall
                length = 6 if "half" not in name else 2
                # Determine the orientation of the wall based on its position
                x = pose.position.x
                y = pose.position.y
                is_vertical = abs(x - min_x) < 1e-2 or abs(x - max_x) < 1e-2
                # Convert the pose to grid coordinates
                x, y = self.pose_to_grid_coordinates(pose, min_x, min_y, length, is_vertical)
                # Fill in the cells in the grid corresponding to the wall
                if is_vertical:  # The wall is vertical
                    grid[y:y+length, x] = 1
                else:  # The wall is horizontal
                    grid[y, x:x+length] = 1

        return grid

    def pose_to_grid_coordinates(self, pose, min_x, min_y, length, is_vertical):
        # Convert a pose to grid coordinates
        x = int(pose.position.x - min_x)
        y = int(pose.position.y - min_y)
        if is_vertical:
            y -= length // 2
        else:
            x -= length // 2
        return x, y

    def get_relative_locations_and_orientations(self):
        # Get model states
        model_states = rospy.wait_for_message('/gazebo/model_states', ModelStates)

        # Find the position and orientation of the robot
        robot_index = model_states.name.index(self.robot_name)
        robot_pose = model_states.pose[robot_index]
        robot_position = np.array([robot_pose.position.x, robot_pose.position.y])
        robot_orientation = euler_from_quaternion([
            robot_pose.orientation.x,
            robot_pose.orientation.y,
            robot_pose.orientation.z,
            robot_pose.orientation.w
        ])[2]  # yaw

        # Calculate the relative locations and orientations of all objects
        relative_locations_and_orientations = []
        for name, pose in zip(model_states.name, model_states.pose):
            if name != self.robot_name:
                # Calculate the relative location
                position = np.array([pose.position.x, pose.position.y])
                relative_location = position - robot_position

                # Calculate the relative orientation
                orientation = euler_from_quaternion([
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w
                ])[2]  # yaw
                relative_orientation = orientation - robot_orientation

                relative_locations_and_orientations.append((relative_location, relative_orientation))

        return relative_locations_and_orientations

def test():
    # Create an instance of GazeboObservationGenerator
    generator = GazeboObservationGenerator('locobot')  # replace 'robot' with the actual name of your robot

    # Generate and print the occupancy grid
    occupancy_grid = generator.get_occupancy_grid()
    print('Occupancy Grid:')
    print(occupancy_grid)

    # Generate and print the relative locations and orientations
    relative_locations_and_orientations = generator.get_relative_locations_and_orientations()
    print('\\nRelative Locations and Orientations:')
    for relative_location, relative_orientation in relative_locations_and_orientations:
        print('Relative Location: {}, Relative Orientation: {}'.format(relative_location, relative_orientation))

if __name__ == '__main__':
    test()