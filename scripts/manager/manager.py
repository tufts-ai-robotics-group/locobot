from planner.Planner import Planner
from learner.Learner import Learner

from enum import Enum

class SymStatus(Enum):
    SUCCESS=0
    PRECONDITION_FAILURE=1
    EFFECT_FAILURE=2

class Manager(object):

    def __init__(self,
                 planner: Planner,
                 learner: Learner,
                 sym_actions: dict) -> None:

        self._planner = planner

        # Verify that there is a symbolic action function for each planner action
        sym_actions_set = set(sym_actions.keys())
        planner_actions_set = set(self._planner.actions)
        if sym_actions_set.difference(planner_actions_set) != set():
            raise ValueError("Error: param 'sym_actions' has different keys than the PDDL action names")
        self._sym_actions = sym_actions


    ######################################### Symbolic Side ###############################################
    def __new_problem(self) -> None:
        self._planner.new_problem()

    def __next_action(self) -> str:
        return self._planner.next_action()

    def __verify_preconditions(self, 
                             *args) -> bool:
        return self._planner.verify_preconditions(args[0], *args[1:])
    
    
    def __verify_effects(self,
                       *args) -> bool:
        
        return self._planner.verify_effects(args[0], *args[1:])

    def __execute_action(self,
                       *args) -> SymStatus:
        
        if not self._planner.verify_preconditions(args[0], *args[1:]):
            return SymStatus.PRECONDITION_FAILURE

        self._sym_actions[args[0]](*args[1:])

        if not self._planner.verify_effects(args[0], *args[1:]):
            return SymStatus.EFFECT_FAILURE
        
        return SymStatus.SUCCESS

    def __get_sym_state(self):
        return self._planner.get_state()
    
    ######################################### Subsymbolic Side ###############################################
    

    def run(self):
        # Initialize problem
        self.__new_problem()

        # Loop through actions in plan 
        while action := self.__next_action():
            status = self.__execute_action(*action)

            # Do RL
            if status == SymStatus.PRECONDITION_FAILURE:
                pass
            elif status == SymStatus.EFFECT_FAILURE:
                pass 
