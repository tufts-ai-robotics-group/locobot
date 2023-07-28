'''
This class implements the observation generator for the environment.
'''

import sys
import os
import itertools

sys.path.append("../knowledge/PDDL/recycle_bot")
sys.path.append("../knowledge/pddl-parser")

import numpy as np
import quaternion
from pddl_parser.PDDL import PDDL_Parser


class ObservationGenerator:
    def __init__(self, domain_file, problem_file):
        # Reference to the environment
        # self.env = env
        self.parser = PDDL_Parser()
        self.parser.parse_domain(domain_file)
        self.parser.parse_problem(problem_file)


    def get_observation(self):
        
        # Sensory data
        # sensory_data = self.get_sensory_data()

        # # Occupancy grid
        # occupancy_grid = self.get_occupancy_grid()

        # # One-hot encoded predicates
        # predicates_vector = self.get_predicates_vector()

        # Combine all parts of the observation
        # observation = (sensory_data, occupancy_grid, predicates_vector)
        observation = self.get_predicates_vector()

        return observation

    def get_sensory_data(self):
        '''
        Needs work.
        '''
        sensory_data = {}
        robot_state = self.env.get_robot_state()
        for obj in self.env.get_all_objects():
            obj_state = self.env.get_object_state(obj)
            relative_state = self.calculate_relative_state(robot_state, obj_state)
            sensory_data[obj] = relative_state
        return sensory_data

    def get_occupancy_grid(self):
        '''
        Needs work.
        '''
        width, height = self.env.get_env_dimensions()
        wall_positions = self.env.get_wall_positions()
        door_positions = self.env.get_door_positions()
        return self.generate_occupancy_grid(width, height, wall_positions, door_positions)


    def calculate_relative_state(self, robot_state, obj_state):
        robot_position, robot_orientation = robot_state
        obj_position, obj_orientation = obj_state

        # Calculate relative position in 3D Cartesian coordinates
        relative_position = [obj_pos - rob_pos for rob_pos, obj_pos in zip(robot_position, obj_position)]

        # Calculate relative orientation in quaternion
        relative_orientation = self.calculate_relative_orientation(robot_orientation, obj_orientation)

        return relative_position, relative_orientation

    def calculate_relative_orientation(self, robot_orientation, obj_orientation):
        # Convert lists to quaternion objects
        robot_quat = quaternion.quaternion(*robot_orientation)
        obj_quat = quaternion.quaternion(*obj_orientation)

        # Calculate the relative orientation
        relative_orientation = robot_quat * obj_quat.inverse()

        # Convert the quaternion object back to a list
        return [relative_orientation.w, relative_orientation.x, relative_orientation.y, relative_orientation.z]

    def generate_occupancy_grid(self, width, height, wall_positions, door_positions):
        # Initialize the grid with all free space
        grid = [[0 for _ in range(width)] for _ in range(height)]

        # Set wall cells to 1
        for (x, y) in wall_positions:
            grid[y][x] = 1

        # Set door cells to 0
        for (x, y) in door_positions:
            grid[y][x] = 0

        return grid
    
    def get_predicates_vector(self):
        predicates_vector = {}
        for predicate in self.parser.predicates:
            print ("predicate: ", predicate)
            predicates_vector[predicate] = self.get_predicate_vector(predicate)
        return predicates_vector

    def get_predicate_vector(self, predicate):
        # Generate a one-hot encoded vector for a given predicate
        predicate_vector = [0] * sum(len(v) for v in self.parser.objects.values())
        for fact in self.parser.state:
            if fact[0] == predicate:
                # Get the list of all objects of this type
                objects_of_type = []
                for object_type, objects in self.parser.objects.items():
                    objects_of_type.extend(objects)
                # Get the index of the object in this list
                try:
                    index = objects_of_type.index(fact[1])
                    # Set the corresponding position in the predicate vector to 1
                    predicate_vector[index] = 1
                except ValueError:
                    print(f"Object {fact[1]} not found in objects list.")
        return predicate_vector
    

# testing
def main():
    # Specify the paths to your domain and problem PDDL files
    print (os.getcwd())
    domain_file = "../knowledge/PDDL/recycle_bot/domain.pddl"
    problem_file = "../knowledge/PDDL/recycle_bot/problem.pddl"  # Replace with your actual problem PDDL file path

    # Create an ObservationGenerator instance
    observation_generator = ObservationGenerator(domain_file, problem_file)

    # Generate an observation
    observation = observation_generator.get_observation()

    # Print the observation
    print(observation)

if __name__ == "__main__":
    main()
