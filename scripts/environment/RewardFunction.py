'''
This class implements the reward function generation for the environment.
'''

import sys
import os
import itertools

sys.path.append("../knowledge/PDDL/recycle_bot")
sys.path.append("../knowledge/pddl-parser")

from pddl_parser.PDDL import PDDL_Parser


class RewardFunction:
    def __init__(self, env, domain_file, problem_file):
        # Reference to the environment
        self.env = env

        # Parse the domain and problem files
        self.parser = PDDL_Parser()
        self.parser.parse_domain(domain_file)
        self.parser.parse_problem(problem_file)

        # Dynamically add reward functions for each action
        for action in self.parser.actions:
            print (action.name)
            self.add_reward_function(action)

    def add_reward_function(self, action):
        def reward_function():
            # Compute the reward based on the state and the effects of the action
            # Check if all add_effects are in the parser's state
            for effect in action.add_effects:
                # print ("effect {} of action {}".format(effect, action.name))
                # print ("current state {}".format(self.parser.state))
                if tuple(effect) not in self.parser.state:
                    return -1  # the effect is not in the state, return a negative reward
            # Check if all del_effects are not in the parser's state
            for effect in action.del_effects:
                if tuple(effect) in self.parser.state:
                    return -1  # the effect is still in the state, return a negative reward
            return 1  # all add_effects are in the state and all del_effects are not in the state, return a positive reward

        # Dynamically add the reward function to the class
        setattr(self, f"{action.name}_reward", reward_function)


    def get_reward(self, action):
        # Call the appropriate reward function based on the action
        reward_function = getattr(self, f"{action.name}_reward", None)
        if reward_function is None:
            raise ValueError(f"Reward function for action '{action.name}' not found")
        return reward_function()

    
def main():
    # Instantiate your environment (this will be specific to your project)
    env = None  # replace with your environment

    # Specify the path to your domain and problem files
    domain_file = "../knowledge/PDDL/recycle_bot/domain.pddl"  
    problem_file = "../knowledge/PDDL/recycle_bot/problem.pddl"  
    # Instantiate the RewardFunction
    reward_function = RewardFunction(env, domain_file, problem_file)

    # Test getting a reward for each action
    for action in reward_function.parser.actions:
        reward = reward_function.get_reward(action)
        print(f"Reward for action '{action.name}': {reward}")

if __name__ == "__main__":
    main()

