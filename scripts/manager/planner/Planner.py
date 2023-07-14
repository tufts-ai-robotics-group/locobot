import re
from os import mkdir
from os.path import exists, join

class Planner:
    """
    
    """
    
    def __init__(self, 
                 domain_file_path: str, 
                 problem_file_directory: str, 
                 problem_file_prefix: str, 
                 predicate_funcs: dict) -> None:
        """
        Initialize the Planner class.

        Args:
            domain_file_path (str): The path to the domain file.
            problem_file_directory (str): The directory to store problem files.
            problem_file_prefix (str): The prefix for the problem file names.
            predicates_funcs (dict): Dictionary with predicate names as keys and predicate functions as values.
            
        Raises:
            Exception: If there is an error opening the domain file.
            ValueError: If predicate keys or their input counts don't match the function dictionary.
        """
        self.domain_file_path = domain_file_path
        self.problem_file_directory = problem_file_directory
        self.problem_file_prefix = problem_file_prefix
        self.file_counter = 0  # Initialize the file counter to 0
        self.predicates = {}  # Dictionary to store predicate names and number of inputs
        self.preconditions = {} # Dictionary to store the action names and the corresponding precondition check function
        self.effects = {} # Dictionary to store the action names and the corresponding effect check function
        try:
            self.domain_file = open(domain_file_path, 'r')
            self.parse_predicates() # Parse the predicates in the domain file, populate self.predicates
            self.verify_predicates(predicate_funcs) # Verify predicates against function dictionary
            self.parse_actions() # Parse the actions in the domain file, populate self.preconditions and self.effects
            self.domain_file.close()

        except IOError:
            raise Exception("Error opening the domain file.")

        self.plan_file_directory = join(self.problem_file_directory, "plan")
        if not exists(self.plan_file_directory):
            try:
                mkdir(self.plan_file_directory)
            except IOError:
                raise Exception("Error creating plan directory")
        
        # Create initial problem and plan
        self.new_problem()

    def parse_predicates(self) -> None:
        """
        Parse the predicates in the domain file and build a dictionary of predicate names and input count.

        This function does not return anything but saves the parsed predicates as a class member.
        """
        predicate_pattern = re.compile(r'\(predicates(.+?)\)', re.DOTALL | re.IGNORECASE)
        predicate_section = re.search(predicate_pattern, self.domain_file.read())

        if predicate_section:
            predicates_text = predicate_section.group(1)
            predicate_lines = predicates_text.strip().split('\n')
            for line in predicate_lines:
                line = line.strip()
                if line.startswith('('):
                    predicate_parts = line[1:-1].split()
                    predicate_name = predicate_parts[0]
                    input_count = len(predicate_parts) - 1
                    self.predicates[predicate_name] = input_count

    def verify_predicates(self, function_dict: dict) -> None:
        """
        Verify that the predicate keys and input counts match the function dictionary.

        Args:
            function_dict (dict): Dictionary with function names as keys and functions as values.

        Raises:
            ValueError: If predicate keys or their input counts don't match the function dictionary.
        """
        predicate_keys = set(self.predicates.keys())
        function_keys = set(function_dict.keys())

        if predicate_keys != function_keys:
            raise ValueError("Predicate keys do not match function dictionary keys.")

        for key, input_count in self.predicates.items():
            function = function_dict[key]
            function_input_count = function.__code__.co_argcount
            if input_count != function_input_count:
                raise ValueError(f"Input count mismatch for predicate '{key}'. Expected {input_count} inputs, got {function_input_count}.")

    def parse_actions(self) -> None:
        """
        Parse the actions in the domain file and build preconditions and effects for each action.

        This function populates the self.preconditions and self.effects dictionaries.
        """
        action_pattern = re.compile(r'\(action(.+?)\)', re.DOTALL | re.IGNORECASE)
        action_sections = re.findall(action_pattern, self.domain_file.read())

        for action_section in action_sections:
            action_lines = action_section.strip().split('\n')
            action_name = None
            parameters = []
            preconditions = []
            effects = []

            for line in action_lines:
                line = line.strip()
                if line.startswith(':parameters'):
                    parameters = [param for param in line.split() if param.startwith("?")]
                if line.startswith(':action'):
                    action_name = line.split()[1]
                elif line.startswith(':precondition'):
                    preconditions = self.parse_condition(line.split(':precondition')[1].strip())
                elif line.startswith(':effect'):
                    effects = self.parse_condition(line.split(':effect')[1].strip())

            if action_name:
                self.preconditions[action_name] = self.build_boolean_function(preconditions, parameters)
                self.effects[action_name] = self.build_boolean_function(effects, parameters)

    def parse_condition(self, condition_text: str) -> list:
        """
        Parse a condition (precondition or effect) and build a corresponding function.

        Args:
            condition_text (str): The text representation of the condition.

        Returns:
            list: The parsed condition as a nested list representing the condition tree.
        """
        conditions = []

        # Split condition text based on 'and', 'or', and 'not' keywords
        condition_parts = re.split(r'\b(and|or|not)\b', condition_text, flags=re.IGNORECASE)
        condition_parts = [part.strip() for part in condition_parts if part.strip()]

        for part in condition_parts:
            if part.lower() == 'and' or part.lower() == 'or':
                conditions.append(part.lower())
            elif part.lower() == 'not':
                conditions.append('not')
            else:
                predicate_name, args = self.parse_predicates(part)
                conditions.append((predicate_name, args))

        return conditions
    
    def build_boolean_function(self, condition: list, params: list):
        """
        Build a boolean function for a parsed condition using predicates.

        Args:
            condition (list): Parsed condition as a nested list.
            params (list): A list of strings representing the PDDL action parameters.

        Returns:
            function: The constructed boolean function.
        """
        if condition[0] == 'and':
            return lambda *args: all(self.build_boolean_function(cond, self.predicates)(*args) for cond in condition[1:])
        elif condition[0] == 'or':
            return lambda *args: any(self.build_boolean_function(cond, self.predicates)(*args) for cond in condition[1:])
        elif condition[0] == 'not':
            return lambda *args: not self.build_boolean_function(condition[1], self.predicates)(*args)
        else:
            predicate_name = condition[0]
            predicate_args = condition[1:]
            p_arg_indecies = [params.index(arg) for arg in predicate_args]
            predicate_function = self.predicates[predicate_name]

            # Only pass relavent arguments to predicate function
            return lambda *args: predicate_function(*[args[index] for index in p_arg_indecies])
    
    def verify_preconditions(self, action: str, *args):
        """
        Checks the preconditions of an action.

        Args:
            action (str): The action name.
            args (list(str)): An arbitrarily long list of string args.

        Returns:
            bool: If the preconditions are satisfied.
        """
        return self.preconditions[action](*args)
    
    def verify_effects(self, action: str, *args):
        """
        Checks the effects of an action.

        Args:
            action (str): The action name.
            args (list(str)): An arbitrarily long list of string args.

        Returns:
            bool: If the effects are satisfied.
        """
        return self.preconditions[action](*args)

    def generate_problem_str(self) -> str:
        """
        User defined function to create a string representation of the problem file.

        Returns:
            str: The problem file as a string.

        Raises:
            NotImplementedError: Subclass responsibility.
        """
        raise NotImplementedError

    def generate_problem_file(self) -> None:
        """
        Creates new problem file based on current state.

        This function saves the problem file to the user specified directory
        with the user specified prefix and an increasing numeric suffix.

        Raises:
            Exception: If there is an error opening or writing to the problem file.

        """
        problem_str = self.generate_problem_str()
        problem_path = join(self.problem_file_directory, f"{self.problem_file_prefix}_{str(self.file_counter)}")
        
        # Open and write to problem file
        try:
            with open(problem_path, "w") as file:
                file.write(problem_str)
                file.close()
        except IOError:
            raise Exception("Error opening/writing to problem file.")
        
    def generate_plan_str(self) -> str:
        """
        User defined function to create a string representation of the plan file.
        Each line of the string must contain one action call. The action call 
        must be white space deliminated, with the first word being the action name, and
        each subsequent word being the parameters to the action in the correct order.

        Retruns:
            str: The plan as a string

        Raises:
            NotImplementedError: Subclass responsibility
        """
        raise NotImplementedError
    
    def create_action_generator(self):
        """
        Creates a generator function for actions in a PDDL plan.
        Sets the 'action_gen' attribute to the generator.
        """
        
        plan_lines = self.plan_str.strip().split('\n')
        actions = []
        for line in plan_lines:
            actions.append(line.split())
        
        self.action_gen = (action for action in actions)
    
    def generate_plan(self):
        """
        Creates new problem file based on current state.

        This function saves the plan file in the plan directory within the user 
        specified problem file directory with the user specified problem prefix and an 
        increasing numeric suffix.

        Raises:
            Exception: If there is an error opening or writing to the plan file.

        """
        self.plan_str = self.generate_plan_str()
        plan_path = join(self.plan_file_directory, f"{self.problem_file_prefix}_{str(self.file_counter)}")
        
        # Open and write to problem file
        try:
            with open(plan_path, "w") as file:
                file.write(self.plan_str)
                file.close()
        except IOError:
            raise Exception("Error opening/writing to plan file.")
        
        self.file_counter += 1

    def new_problem(self):
        """
        Constructs a new problem and plan at the current state.

        Raises: 
            NotImplementedError: If subclass responsibiliy has not been fulfilled.
            Exception: If IOError occurs.
        """
        self.generate_problem_file()
        self.generate_plan()
        self.create_action_generator() 

    def __del__(self) -> None:
        """
        Destructor method to clean up resources.

        Closes the opened domain file.
        """
        if hasattr(self, 'domain_file') and not self.domain_file.closed:
            self.domain_file.close()

    