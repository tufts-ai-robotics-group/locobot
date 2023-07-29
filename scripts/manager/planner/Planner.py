import re
from os import mkdir
from os.path import exists, join

class Planner:
    """
    Parent planner class for symbolic planning aspect of RaPID Learn paradigm.

    Attributes:
        domain_path (str): Path to the domain file (read-only).
        problem_dir (str): Path to the directory in which generated problem files are stored.
        probelm_prefix (str): The prefix string used for naming new problem files.
        actions (str): List of actions in the PDDL

    Functions:
        verify_preconditions ((str, list(str))->bool): Checks if the preconditions of an action are satisfied given the
            current state and a list of arguments to the action.
        verify_effects ((str, list(str))->bool): Checks the if the effects of an action are satisfied given the
            current state and a list of arguments to the action.
    """

    def __init__(self, 
                 domain_path: str, 
                 problem_dir: str, 
                 problem_prefix: str, 
                 predicate_funcs: dict) -> None:
        """
        Initialize the Planner class. 

        Args:
            domain_path (str): The path to the domain file.
            problem_dir (str): The directory to store problem files.
            problem_prefix (str): The prefix for the problem file names.
            predicates_funcs (dict): Dictionary with predicate names as keys and predicate functions as values.
                The functions must have the same number of arguments as their corresponding predicate in the PDDL.
            
        Raises:
            Exception: If there is an error opening the domain file.
            ValueError: If predicate keys or their input counts don't match the function dictionary.
        """
        self._domain_path = domain_path
        self._problem_dir = problem_dir
        self._problem_prefix = problem_prefix
        self._file_counter = 0  # Initialize the file counter to 0
        self._actions = []
        self._predicates = {}  # Dictionary to store predicate names and number of inputs
        self._preconditions = {} # Dictionary to store the action names and the corresponding precondition check function
        self._effects = {} # Dictionary to store the action names and the corresponding effect check function
        try:
            self._domain_file = open(domain_path, 'r')
            self.__parse_predicates() # Parse the predicates in the domain file, populate self._predicates
            self.__verify_predicates(predicate_funcs) # Verify predicates against function dictionary
            self.__parse_actions() # Parse the actions in the domain file, populate self._preconditions and self._effects
            self._domain_file.close()

        except IOError:
            raise Exception("Error opening the domain file.")

        self.plan_file_directory = join(self.problem_dir, "plan")
        if not exists(self.plan_file_directory):
            try:
                mkdir(self.plan_file_directory)
            except IOError:
                raise Exception("Error creating plan directory")

    ######################################### Getters and Setters ###############################################
    @property
    def domain_path(self) -> str:
        """
        Getter for domain_path.
        """
        return self._domain_path
    
    @property
    def problem_dir(self) -> str:
        """
        Getter for problem_dir.
        """
        return self._problem_dir
    
    @problem_dir.setter
    def problem_dir(self, 
                    new_dir) -> None:
        """
        Setter for problem_dir.
        """
        self.problem_dir = new_dir
    
    @property
    def problem_prefix(self) -> str:
        """
        Getter for problem_prefix.
        """
        return self._problem_dir
    
    @problem_prefix.setter
    def problem_prefix(self, 
                       new_prefix) -> None:
        """
        Setter for problem_prefix.
        """
        self.problem_prefix = new_prefix

    @property
    def actions(self) -> str:
        """
        Getter for actions
        """
        return self._actions


    ######################################### Public Functions ###############################################
    def verify_preconditions(self, 
                             action: str, 
                             *args):
        """
        Checks the preconditions of an action.

        Args:
            action (str): The action name.
            args (list(str)): An arbitrarily long list of string args.

        Returns:
            bool: If the preconditions are satisfied.
        """
        return self._preconditions[action](*args)
    
    def verify_effects(self, 
                       action: str, 
                       *args):
        """
        Checks the effects of an action.

        Args:
            action (str): The action name.
            args (list(str)): An arbitrarily long list of string args.

        Returns:
            bool: If the effects are satisfied.
        """
        return self._preconditions[action](*args)
    
    def new_problem(self):
        """
        Constructs a new problem and plan at the current state.

        Raises: 
            NotImplementedError: If subclass responsibiliy has not been fulfilled.
            Exception: If IOError occurs.
        """
        self.__generate_problem()
        self.__generate_plan()
        self.__create_action_generator() 
    

    def next_action(self) -> str:
        """
        Gets the next action in the plan.

        Returns:
            str: The next action in a plan, if available. If not available,
            returns None.
        """

        try:
            return next(self._action)
        except StopIteration:
            return None

    def get_state(self):
            """
            User defined representation of the symbolic state of the environment.

            Raises:
                NotImplementedError: Subclass responsibility
            """
            raise NotImplementedError

    
    ######################################### Private Functions ###############################################

    def __parse_predicates(self) -> None:
        """
        Parse the predicates in the domain file and build a dictionary of predicate names and input count.

        This function does not return anything but saves the parsed predicates as a class member.
        """
        predicate_pattern = re.compile(r'\(predicates(.+?)\)', re.DOTALL | re.IGNORECASE)
        predicate_section = re.search(predicate_pattern, self._domain_file.read())

        if predicate_section:
            predicates_text = predicate_section.group(1)
            predicate_lines = predicates_text.strip().split('\n')
            for line in predicate_lines:
                line = line.strip()
                if line.startswith('('):
                    predicate_parts = line[1:-1].split()
                    predicate_name = predicate_parts[0]
                    input_count = len(predicate_parts) - 1
                    self._predicates[predicate_name] = input_count

    def __verify_predicates(self, 
                            function_dict: dict) -> None:
        """
        Verify that the predicate keys and input counts match the function dictionary.

        Args:
            function_dict (dict): Dictionary with function names as keys and functions as values.

        Raises:
            ValueError: If predicate keys or their input counts don't match the function dictionary.
        """
        predicate_keys = set(self._predicates.keys())
        function_keys = set(function_dict.keys())

        if predicate_keys != function_keys:
            raise ValueError("Predicate keys do not match function dictionary keys.")

        for key, input_count in self._predicates.items():
            function = function_dict[key]
            function_input_count = function.__code__.co_argcount
            if input_count != function_input_count:
                raise ValueError(f"Input count mismatch for predicate '{key}'. Expected {input_count} inputs, got {function_input_count}.")

    def __parse_actions(self) -> None:
        """
        Parse the actions in the domain file and build preconditions and effects for each action.

        This function populates the self._preconditions and self._effects dictionaries.
        """
        action_pattern = re.compile(r'\(action(.+?)\)', re.DOTALL | re.IGNORECASE)
        action_sections = re.findall(action_pattern, self._domain_file.read())

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
                    preconditions = self.__parse_condition(line.split(':precondition')[1].strip())
                elif line.startswith(':effect'):
                    effects = self.__parse_condition(line.split(':effect')[1].strip())

            if action_name:
                self._action.append(action_name)
                self._preconditions[action_name] = self.__build_boolean_function(preconditions, parameters)
                self._effects[action_name] = self.__build_boolean_function(effects, parameters)

    def __parse_condition(self, 
                          condition_text: str) -> list:
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
                predicate_name, args = self.__parse_predicates(part)
                conditions.append((predicate_name, args))

        return conditions
    
    def __build_boolean_function(self, 
                                 condition: list, 
                                 params: list):
        """
        Build a boolean function for a parsed condition using predicates.

        Args:
            condition (list): Parsed condition as a nested list.
            params (list): A list of strings representing the PDDL action parameters.

        Returns:
            function: The constructed boolean function.
        """
        if condition[0] == 'and':
            return lambda *args: all(self.__build_boolean_function(cond, self._predicates)(*args) for cond in condition[1:])
        elif condition[0] == 'or':
            return lambda *args: any(self.__build_boolean_function(cond, self._predicates)(*args) for cond in condition[1:])
        elif condition[0] == 'not':
            return lambda *args: not self.__build_boolean_function(condition[1], self._predicates)(*args)
        else:
            predicate_name = condition[0]
            predicate_args = condition[1:]
            p_arg_indecies = [params.index(arg) for arg in predicate_args]
            predicate_function = self._predicates[predicate_name]

            # Only pass relavent arguments to predicate function
            return lambda *args: predicate_function(*[args[index] for index in p_arg_indecies])

    def __generate_problem_str(self) -> str:
        """
        User defined function to create a string representation of the problem file. 
        Creates string being problem file contents based on current state.

        Returns:
            str: The problem file as a string.

        Raises:
            NotImplementedError: Subclass responsibility.
        """
        raise NotImplementedError

    def __generate_problem(self) -> None:
        """
        This function saves the problem file to the user specified directory
        with the user specified prefix and an increasing numeric suffix.

        Raises:
            Exception: If there is an error opening or writing to the problem file.

        """
        problem_str = self.__generate_problem_str()
        problem_path = join(self.problem_dir, f"{self.problem_prefix}_{str(self._file_counter)}")
        
        # Open and write to problem file
        try:
            with open(problem_path, "w") as file:
                file.write(problem_str)
                file.close()
        except IOError:
            raise Exception("Error opening/writing to problem file.")
        
    def __generate_plan_str(self) -> str:
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
    
    def __create_action_generator(self):
        """
        Creates a generator function for actions in a PDDL plan.
        Sets the 'action_gen' attribute to the generator.
        """
        
        plan_lines = self.plan_str.strip().split('\n')
        actions = []
        for line in plan_lines:
            actions.append(line.split())
        
        self._action = (action for action in actions)
    
    def __generate_plan(self):
        """
        Creates new problem file based on current state.

        This function saves the plan file in the plan directory within the user 
        specified problem file directory with the user specified problem prefix and an 
        increasing numeric suffix.

        Raises:
            Exception: If there is an error opening or writing to the plan file.

        """
        self.plan_str = self.__generate_plan_str()
        plan_path = join(self.plan_file_directory, f"{self.problem_prefix}_{str(self._file_counter)}")
        
        # Open and write to problem file
        try:
            with open(plan_path, "w") as file:
                file.write(self.plan_str)
                file.close()
        except IOError:
            raise Exception("Error opening/writing to plan file.")
        
        self._file_counter += 1

    def __del__(self) -> None:
        """
        Destructor method to clean up resources.

        Closes the opened domain file.
        """
        if hasattr(self, '_domain_file') and not self._domain_file.closed:
            self._domain_file.close()

    