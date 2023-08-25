
import sys

sys.path.append("../knowledge/PDDL/recycle_bot")
sys.path.append("../knowledge/pddl-parser")

from pddl_parser.PDDL import PDDL_Parser

class RewardFunction:
    def __init__(self, domain_file, problem_file, failed_action):
        # Parse the domain and problem files
        self.parser = PDDL_Parser()
        self.parser.parse_domain(domain_file)
        self.parser.parse_problem(problem_file)
        
        # Extract the effects of the failed action
        self.failed_action_effects = self.extract_action_effects(failed_action)

    def extract_action_effects(self, action_name):
        # Loop through the actions to find the specified action
        for action in self.parser.actions:
            if action.name == action_name:
                # Convert frozensets to sets and then merge them
                return set(action.add_effects) | set(action.del_effects)
        return set()

    def get_reward(self, success, executed_info, problem_file):

        # Check if the action was successful
        if not success:
            return -1, False, False  # Negative reward as the action failed
        else: # in the case when the aciton was successful
            # Parse the current state
            self.parser.parse_problem(problem_file)

            # Check if all action effects are satisfied in the current state
            for effect in self.failed_action_effects:
                if tuple(effect) not in self.parser.state:
                    return -1, False, False  # Negative reward as the effect is not satisfied
            return 1, True, True  # Positive reward as all effects are satisfied

# test function        
def test():
    # Specify the path to your domain and problem files
    domain_file = "/home/bharatkesari/interbotix_ws/src/locobot_custom/scripts/knowledge/PDDL/recycle_bot/domain.pddl"
    problem_file = "/home/bharatkesari/interbotix_ws/src/locobot_custom/scripts/knowledge/PDDL/recycle_bot/problem.pddl"

    failed_action = "(approach doorway_1 room_1 ball_1)"
    
    rf = RewardFunction(domain_file, problem_file, failed_action)
    reward, _, _ = rf.get_reward(success = True, executed_info= {}, problem_file = problem_file)
    print(f"Reward based on the failed action '{failed_action}' and current state: {reward}")

if __name__ == "__main__":
    test()