from planner.Planner import Planner


from enum import Enum

class SymStatus(Enum):
    SUCCESS=0
    PRECONDITION_FAILURE=1
    EFFECT_FAILURE=2

class Agent(object):

    def __init__(self,
                 planner: Planner,
                 sym_actions: dict) -> None:

        self.planner = planner

        # Verify that there is a symbolic action function for each planner action
        sym_actions_set = set(sym_actions.keys())
        planner_actions_set = set(self.planner.actions)
        if sym_actions_set.difference(planner_actions_set) != set():
            raise ValueError("Error: param 'sym_actions' has different keys than the PDDL action names")
        self._sym_actions = sym_actions


    ######################################### Symbolic Side ###############################################
    def new_problem(self) -> None:
        self.planner.new_problem()

    def next_action(self) -> str:
        return self.planner.next_action()

    def verify_preconditions(self, 
                             *args) -> bool:
        return self.planner.verify_preconditions(args[0], *args[1:])
    
    
    def verify_effects(self,
                       *args) -> SymStatus:
        
        self.planner.verify_effects(args[0], *args[1:])
        
    def execute_action(self,
                       *args):
        if not self.verify_preconditions(args[0], *args[1:]):
            return SymStatus.PRECONDITION_FAILURE

        self._sym_actions[args[0]](*args[1:])

        if not self.verify_effects(args[0], *args[1:]):
            return SymStatus.EFFECT_FAILURE
        
        return SymStatus.SUCCESS
    


    
    ######################################### Subsymbolic Side ###############################################
    def load_and_run_executor(self, 
                              action: str):
        raise NotImplementedError
         

    def learn_executor(self,
                       action: str):
        raise NotImplementedError

    def run(self):
        # Initialize problem
        self.new_problem()

        # Loop through actions in plan 
        while action := self.next_action():
            print ("action: ", *action)
            status = self.execute_action(*action)
            print ("status: ", status)
            
            if status != SymStatus.SUCCESS:
                self.learn_executor(action)
            


