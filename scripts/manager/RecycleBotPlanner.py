from planner.Planner import Planner
from pddl_parser.planner import Planner as PDDL_Planner 
from os.path import join

class RecycleBotPlanner(Planner):

    def __init__(self,
                 domain_path: str,
                 problem_dir: str,
                 problem_prefix: str,
                 predicate_funcs: dict,
                 objects: dict) -> None:
        
        super().__init__(domain_path, problem_dir, problem_prefix, predicate_funcs)
        self.objects = objects
        self.planner = PDDL_Planner()
    
    def generate_problem_str(self) -> str:

        def get_at(obj: str):
            for room in self.objects['room']:
                if (self._predicate_funcs['at'](room, obj)):
                    return f"(at {room} {obj})"

        def get_facing():
            for obj_type in self.objects.keys():
                for obj in self.objects[obj_type]:
                    if (self._predicate_funcs['facing'](obj)):
                        return f"(facing {obj})"
                
        
        def get_hold():
            for obj_type in self.objects.keys():
                for obj in self.objects[obj_type]:
                    if (self._predicate_funcs['hold'](obj)):
                        return f"(hold {obj})"

        robot_1_at = get_at('robot_1')
        can_1_at = get_at('can_1')
        ball_1_at = get_at('ball_1')
        bin_1_at = get_at('bin_1')
        facing = get_facing()
        hold = get_hold()

        return f"""(define (problem recycle) (:domain recycle_bot)
(:objects 
    doorway_1 - doorway
    room_1 room_2 - room
    ball_1 - ball
    can_1 - can
    bin_1 - bin
    nothing - nothing
    robot_1 - robot
)

(:init
    ; always true states
    (connect room_1 room_2 doorway_1)
    (connect room_2 room_1 doorway_1)
    (at room_1 doorway_1)
    (at room_2 doorway_1)

    ; variable states
    {robot_1_at}
    {can_1_at}
    {ball_1_at}
    {bin_1_at}
    {facing}
    {hold}
)

(:goal (and
    ; (contain trash_1 bin_1)
    ; (facing trash_1)
    ; (at room_2 robot_1)
    ; (hold trash_1)
    ; (facing bin_1)
    (contain ball_1 bin_1)
    ; (facing doorway_1)
    )
)
)"""

    def __get_plan(self):
        # Get the plan from the planner
        plan = self.planner.solve(self._domain_path, join(self._problem_dir, f"{self.problem_prefix}_{str(self._file_counter)}.pddl"))
        if plan is not None:
            return plan
        else:
            return None

    def generate_plan_str(self) -> str:
        plan = self.__get_plan()
        plan_str = ""
        for act in plan:
            plan_str += (act.name + ' ' + ' '.join(act.parameters)+'\n')

        return plan_str