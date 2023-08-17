import sys
from itertools import product

# Add the directories containing the domain and problem files to the system path
sys.path.append("../knowledge/PDDL/recycle_bot")
sys.path.append("../knowledge/pddl-parser")

# Import the PDDL parser
from pddl_parser.PDDL import PDDL_Parser

class ActionSpaceGenerator:
    def __init__(self, domain_file, problem_file):
        """
        Initialize the ActionSpaceGenerator.
        It parses the domain and problem file and generates the grounded actions.

        Args:
        domain_file (str): Path to the PDDL domain file
        problem_file (str): Path to the PDDL problem file
        """
        # Create a PDDL Parser object
        self.parser = PDDL_Parser()

        # Parse the domain and problem files
        self.parser.parse_domain(domain_file)
        self.parser.parse_problem(problem_file)

        # Generate non grounded actions
        self.non_grounded_actions = self.generate_nongrounded_actions()

        # Generate the grounded actions
        self.grounded_actions = self.generate_grounded_actions()

    def generate_nongrounded_actions(self):
        """
        Generate a dictionary of non-grounded actions.

        Returns:
        nongrounded_actions (dict): Dictionary of non-grounded actions, with action names as keys and parameters as values.
        """
        nongrounded_actions = {}
        for action in self.parser.actions:
            nongrounded_actions[action.name] = action.parameters
        return nongrounded_actions


    def generate_grounded_actions(self):
        """
        Generate all possible grounded actions based on the given domain and problem file.

        Returns:
        grounded_actions (list): List of all possible grounded actions as strings.
        """
        grounded_actions = []

        # Iterate over all actions in the domain
        for action in self.parser.actions:
            # Generate all possible combinations of parameters for the current action
            param_combinations = self.generate_param_combinations(action)

            # Iterate over all combinations of parameters
            for combination in param_combinations:
                # Create a grounded action by joining the action name with the objects
                grounded_action = ' '.join([action.name] + combination)

                # Add the grounded action to the list
                grounded_actions.append(grounded_action)

        return grounded_actions

    def generate_param_combinations(self, action):
        """
        Generate all possible combinations of parameters for a given action.

        Args:
        action (Action): The action for which the parameter combinations should be generated.

        Returns:
        param_combinations (list): A list of lists with all possible combinations of parameters for the action.
        """
        param_combinations = []

        # Iterate over all parameters of the action
        for param_name, param_type in action.parameters:
            # Get all objects of the parameter type
            possible_values = self.get_objects_of_type(param_type)

            # If there are no objects of the parameter type, skip this action
            if not possible_values:
                continue

            # Add the objects to the parameter combinations
            param_combinations.append(possible_values)

        # Return all possible combinations of parameters
        return [list(x) for x in product(*param_combinations)]

    def get_objects_of_type(self, object_type):
        """
        Get all objects of a given type.

        Args:
        object_type (str): The type of the objects that should be returned.

        Returns:
        objects (list): A list of all objects of the given type.
        """
        # Get all objects that are directly of the type
        direct_objects = self.parser.objects.get(object_type, [])

        # Get all objects that are a subtype of the type
        subtype_objects = []
        for subtype in self.parser.types.get(object_type, []):
            subtype_objects.extend(self.parser.objects.get(subtype, []))

        return direct_objects + subtype_objects

    def find_parent_type(self, object_type):
        """
        Find the parent type of a given object type.

        Args:
        object_type (str): The object type for which the parent type should be found.

        Returns:
        parent_type (str or None): The parent type of the given object type or None if it doesn't have a parent.
        """
        # Iterate over all types and their subtypes
        for type, subtypes in self.parser.types.items():
            # If the object_type is a subtype of the current type, return it
            if object_type in subtypes:
                return type

        # If the object_type is not a subtype of any type, return None
        return None
    
    def reset(self):
        pass # dummy for now
    
    # def get_valid_actions(self, state):
    #     """
    #     Generate a list of valid actions based on the current state of the world.

    #     Args:
    #     state (set): The current state of the world, represented as a set of predicates.

    #     Returns:
    #     valid_actions (list): List of valid actions in the current state.
    #     """
    #     valid_actions = []
    #     for action in self.grounded_actions:
    #         action_name, *action_objects = action.split()
    #         action_obj = self.get_action_by_name(action_name)
    #         if all(self.check_precondition(pre, action_obj.parameters, action_objects, state) for pre in action_obj.positive_preconditions):
    #             valid_actions.append(action)
    #     return valid_actions

    # def get_action_by_name(self, action_name):
    #     """
    #     Get an action object by its name.

    #     Args:
    #     action_name (str): The name of the action.

    #     Returns:
    #     action (Action): The action object with the given name, or None if no such action exists.
    #     """
    #     for action in self.parser.actions:
    #         if action.name == action_name:
    #             return action
    #     return None

    # def check_precondition(self, precondition, parameters, action_objects, state):
    #     """
    #     Check whether a precondition is satisfied in the current state.

    #     Args:
    #     precondition (tuple): The precondition to check.
    #     parameters (list): The parameters of the action.
    #     action_objects (list): The objects involved in the action.
    #     state (set): The current state of the world.

    #     Returns:
    #     satisfied (bool): True if the precondition is satisfied in the current state, False otherwise.
    #     """
    #     predicate, *pre_vars = precondition
    #     ground_precondition = [predicate] + [self.ground_variable(var, parameters, action_objects) for var in pre_vars]
    #     return tuple(ground_precondition) in state

    # def ground_variable(self, var, parameters, action_objects):
    #     """
    #     Replace a variable with its corresponding object.

    #     Args:
    #     var (str): The variable to replace.
    #     parameters (list): The parameters of the action.
    #     action_objects (list): The objects involved in the action.

    #     Returns:
    #     object (str): The object corresponding to the variable, or None if no such object exists.
    #     """
    #     for param, obj in zip(parameters, action_objects):
    #         if param[0] == var:
    #             return obj
    #     return None
    
    

if __name__ == "__main__":
    domain_file = "../knowledge/PDDL/recycle_bot/domain.pddl"  
    problem_file = "../knowledge/PDDL/recycle_bot/problem.pddl"  

    # Create an ActionSpaceGenerator
    action_gen = ActionSpaceGenerator(domain_file, problem_file)

    # Print all grounded actions
    print("Grounded actions:")
    print (len(action_gen.grounded_actions))
    for action in action_gen.grounded_actions:
        print(action)

    print("\nNon-grounded actions:")
    for action, params in action_gen.non_grounded_actions.items():
        print(f"{action} {params}")

    # # Print all valid actions in the initial state
    # print("\nValid actions in initial state:")
    # valid_actions = action_gen.get_valid_actions(action_gen.parser.state)
    # for action in valid_actions:
    #     print(action)

