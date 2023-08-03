'''
This class implements the observation generator for the environment.
'''

import sys
import os
import itertools
import time

sys.path.append("../knowledge/PDDL/recycle_bot")
sys.path.append("../knowledge/pddl-parser")

import numpy as np
from pddl_parser.PDDL import PDDL_Parser

from ObservationUtils import GazeboObservationGenerator


class ObservationGenerator:
    def __init__(self, domain_file, problem_file):
        # Reference to the environment
        # self.env = env
        self.parser = PDDL_Parser()
        self.parser.parse_domain(domain_file)
        self.parser.parse_problem(problem_file)
        # self.observation_size = self.get_observation_size()
        # self.observation = self.get_observation()

    # def _get_problem_file(self):
    #     generate_new_problem_file
    #     return self.parser.problem_file


    def get_observation(self):
        # Create an instance of GazeboObservationGenerator
        # gazebo_generator = GazeboObservationGenerator()  # replace 'robot' with the actual name of your robot

        # # Get the occupancy grid
        # occupancy_grid = gazebo_generator.get_occupancy_grid()

        # # Get the relative locations and orientations
        # relative_locations_and_orientations = gazebo_generator.get_relative_locations_and_orientations()

        # # Flatten the occupancy grid and the relative locations and orientations
        # flattened_occupancy_grid = occupancy_grid.flatten()
        # flattened_relative_locations_and_orientations = np.array(relative_locations_and_orientations).flatten()

        # # Concatenate the flattened occupancy grid, the flattened relative locations and orientations,
        # # and the predicate vector
        # observation = np.concatenate([flattened_occupancy_grid, flattened_relative_locations_and_orientations, self.get_predicate_vector()])

        observation = self.get_predicates_vectors()
        return observation
    
    def get_predicates_vectors(self):

        facable_objects = self.parser.types['facable'] #get the types
        facable_object_instances = [instance for faceable in facable_objects for instance in self.parser.objects.get(faceable, [])]

        holdable_objects = self.parser.types['holdable']
        holdable_object_instances = [instance for holdable in holdable_objects for instance in self.parser.objects.get(holdable, [])]

        robots = self.parser.objects['robot']
        
        rooms = self.parser.objects['room']

        facing_vector = [0] * len(facable_object_instances)
        holding_vector = [0] * len(holdable_object_instances)
        at_vector = [0] * len(rooms)

        for predicate in self.parser.state:
            if predicate[0] == 'facing':
                object_index = facable_object_instances.index(predicate[1])
                facing_vector[object_index] = 1
            elif predicate[0] == 'hold':
                object_index = holdable_object_instances.index(predicate[1])
                holding_vector[object_index] = 1
            elif predicate[0] == 'at' and predicate[2] in robots:
                room_index = rooms.index(predicate[1])
                at_vector[room_index] = 1
        # print ("facing_vector: ", facing_vector)
        # print ("holding_vector: ", holding_vector)
        # print ("at_vector: ", at_vector)

        return facing_vector + holding_vector + at_vector
    
    def get_observation_space_size(self):
        self.observation_size = len(self.get_observation())
        return self.observation_size
    
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
