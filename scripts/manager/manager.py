from planner.Planner import Planner

class Manager(object):

    def __init__(self,
                 planner: Planner,
                 sym_actions: dict) -> None:

        self._planner = planner

        # Verify that there is a symbolic action function for each planner action
        sym_actions_set = set(sym_actions.keys())
        planner_actions_set = set(self._planner.actions)
        if sym_actions_set.difference(planner_actions_set) != set():
            raise ValueError(f"Error: param 'sym_actions' has different keys than the PDDL action names")
        self._sym_actions = sym_actions


    def new_problem(self) -> None:
        self._planner.new_problem()

    def next_action(self) -> str:
        return self._planner.next_action()

    def verify_preconditions(self, 
                             *args) -> bool:
        return self._planner.verify_preconditions(args[0], *args[1:])
    
    
    def verify_effects(self,
                       *args) -> bool:
        
        return self._planner.verify_effects(args[0], *args[1:])

    def execute_action(self,
                       *args) -> bool:
        
        if not self._planner.verify_preconditions(args[0], *args[1:]):
            return False

        self._sym_actions[args[0]](*args[1:])

        if not self._planner.verify_effects(args[0], *args[1:]):
            return False
        
        return True
    
