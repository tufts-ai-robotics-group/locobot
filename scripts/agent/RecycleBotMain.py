from PDDLActions import RecyleBotPDDLActions
from PDDLPredicates import RecyleBotPDDLPredicates
from RecycleBotAgent import RecycleBotAgent
from RecycleBotPlanner import RecycleBotPlanner 

predicates = RecyleBotPDDLPredicates()
predicates_dict = {
    'at': predicates.at,
    'facing': predicates.facing,
    'connect': predicates.connect,
    'hold': predicates.hold,
    'contain': predicates.contain
}

sym_actions = RecyleBotPDDLActions()
sym_actions_dict = {
    'approach': sym_actions.approach,
    'pass_through_door': sym_actions.pass_through_door,
    'pick': sym_actions.pick,
    'place': sym_actions.place
}

obj = {
    'doorway': ['doorway_1'],
    'room': ['room_1', 'room_2'],
    'ball': ['ball_1'],
    'can': ['can_1'],
    'bin': ['bin_'],
    'nothing': ['nothing'],
    'robot': ['robot_1']
}

domain_path = "/home/bharatkesari/interbotix_ws/src/locobot_custom/scripts/knowledge/PDDL/recycle_bot/domain.pddl"
problem_dir = "/home/bharatkesari/interbotix_ws/src/locobot_custom/scripts/knowledge/PDDL/recycle_bot/problem"
executor_dir = "/home/bharatkesari/interbotix_ws/src/locobot_custom/scripts/knowledge/PDDL/recycle_bot/executor"


planner = RecycleBotPlanner(domain_path, problem_dir, "recycle_bot_problem", predicates_dict, obj)
print (planner.generate_plan_str())

# Now the planner has planned. We need to execute the plan.
# and if during execution if any action fails, we need to learn the executor for that action.


agent = RecycleBotAgent(planner, sym_actions_dict, executor_dir, 100)



if __name__ == "__main__":
    agent.run()