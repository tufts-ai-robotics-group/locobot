'''
This class implements the reward function generation for the environment.
'''

import sys
import os
import itertools
import re
import time

sys.path.append("../knowledge/PDDL/recycle_bot")
sys.path.append("../knowledge/pddl-parser")

from pddl_parser.PDDL import PDDL_Parser
from pddl_parser.planner import Planner



class RewardFunction:
    def __init__(self, env, domain_file, problem_file, failed_action):
        # Reference to the environment
        self.env = env

        # Parse the domain and problem files
        self.parser = PDDL_Parser()
        self.planner = Planner()
        self.parser.parse_domain(domain_file)
        self.parser.parse_problem(problem_file)
        self.plan = self.get_plan(domain_file, problem_file)        
        self.failed_action = failed_action

        # Dynamically add reward functions for each action
        for action in self.parser.actions:
            print (action.name)
            self.add_reward_function(action)

    def add_reward_function(self, action):
        def reward_function(failed_action):
            # Compute the reward based on the state and the effects of the action
            # Check if all add_effects are in the parser's state
            for effect in action.add_effects:
                if tuple(effect) not in self.parser.state:
                    return -1, False  # the effect is not in the state, return a negative reward
            # Check if all del_effects are not in the parser's state
            for effect in action.del_effects:
                if tuple(effect) in self.parser.state:
                    return -1, False  # the effect is still in the state, return a negative reward
            
            # Check if the current state is in the list of plannable states
            plannable_states = self.find_plannable_states(self.parsed_plan, failed_action)
            if self.parser.state in plannable_states:
                return 1, True  # the current state is a plannable state, return a positive reward

            return -1, False  # the current state is not a plannable state, return a negative reward

        # Dynamically add the reward function to the class
        setattr(self, f"{action.name}_reward", reward_function)


    def get_reward(self, action, failed_action):
        # Call the appropriate reward function based on the action
        reward_function = getattr(self, f"{action.name}_reward", None)
        if reward_function is None:
            raise ValueError(f"Reward function for action '{action.name}' not found")
        return reward_function(failed_action)
    

    # # this needs more testing and implementations
    # def find_plannable_states(self, current_plan, failed_action):
    #     current_state = self.parser.state
    #     new_states = set()

    #     for action in reversed(current_plan):
    #         if action == failed_action:
    #             # scenario 1: ignore the failed action
    #             continue

    #             # scenario 2: replace the failed action
    #             # find all applicable actions in the current state
    #             applicable_actions = self.find_applicable_actions(current_state)
    #             for applicable_action in applicable_actions:
    #                 new_state = self.apply_action_effects(current_state, applicable_action)
    #                 new_states.add(new_state)
    #         else:
    #             new_state = self.apply_action_effects(current_state, action)
    #             current_state = new_state  # update the current state
    #             new_states.add(new_state)

    #     return new_states

    # def find_applicable_actions(self, state):
    #     applicable_actions = []
    #     for action in self.parser.actions:
    #         if self.check_preconditions(state, action):
    #             applicable_actions.append(action)
    #     return applicable_actions

    # def apply_action_effects(self, state, action):
    #     new_state = state.copy()
    #     for effect in action.add_effects:
    #         new_state.add(tuple(effect))
    #     for effect in action.del_effects:
    #         new_state.discard(tuple(effect))
    #     return frozenset(new_state)

    # def check_preconditions(self, state, action):
    #     for precondition in action.positive_preconditions:
    #         if tuple(precondition) not in state:
    #             return False
    #     for precondition in action.negative_preconditions:
    #         if tuple(precondition) in state:
    #             return False
    #     return True
    
    def get_plan(self, domain_file, problem_file):
        # Get the plan from the planner
        start_time = time.time()
        plan = self.planner.solve(domain_file, problem_file)
        print('Time: ' + str(time.time() - start_time) + 's')
        if plan is not None:
            print('plan:')
            for act in plan:
                print(act.name + ' ' + ' '.join(act.parameters))
        else:
            sys.exit('No plan was found')
        return plan

# Test case and Example use case of the class
def main():
    # Instantiate your environment (this will be specific to your project)
    env = None  # replace with your environment

    # Specify the path to your domain and problem files
    domain_file = "../knowledge/PDDL/recycle_bot/domain.pddl"  
    problem_file = "../knowledge/PDDL/recycle_bot/problem.pddl"   

    failed_action = "(approach doorway_1 room_1 ball_1)"

    # Instantiate the RewardFunction
    reward_function = RewardFunction(env, domain_file, problem_file, failed_action)

    # Test getting a reward for each action
    for action in reward_function.parser.actions:
        # replace with your failed action
        failed_action = "(approach doorway_1 room_1 ball_1)"

        reward, done = reward_function.get_reward(action, failed_action)
        print(f"Reward for action '{action.name}': {reward}, Done: {done}")

if __name__ == "__main__":
    main()
