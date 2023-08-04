from RecycleBotPlanner import RecycleBotPlanner
from pddl_parser.PDDL import PDDL_Parser
from Manager import Manager

predicates = {
    'at': lambda x, y : True,
    'facing': lambda x : True,
    'connect': lambda x, y, z: True,
    'hold': lambda x: True,
    'contain': lambda x, y: True
}

sym_actions = {
    'approach': lambda x, y, z : True,
    'pass_through_door': lambda x, y, z : True,
    'pick': lambda  x, y : True,
    'place': lambda x, y, z: True
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

domain_path = "/Users/bharatkesari/dev/tufts/locobot/scripts/knowledge/PDDL/recycle_bot/domain.pddl"
problem_dir = "/Users/bharatkesari/dev/tufts/locobot/scripts/knowledge/PDDL/recycle_bot/problem"

pass

planner = RecycleBotPlanner(domain_path, problem_dir, "problem", predicates, obj)
manager = Manager(planner, sym_actions)
manager.run()