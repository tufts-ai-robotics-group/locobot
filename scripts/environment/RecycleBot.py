import rospy

import gym
from gym import spaces

# Main modules
from ObservationGenerator import ObservationGenerator
from RewardFunction import RewardFunction
from ActionGenerator import ActionSpaceGenerator

# ROS related modules
from locobot.srv import Approach, Grasp, Place

class RecycleBot(gym.Env):
    def __init__(self, knowledge_base):
        super(RecycleBot, self).__init__()

        self.observation_generator = ObservationGenerator(knowledge_base.domain_file, knowledge_base.problem_file)
        self.reward_function_generator = RewardFunction(knowledge_base.domain_file, knowledge_base.problem_file, knowledge_base.failed_action)
        self.action_space_generator = ActionSpaceGenerator(knowledge_base.domain_file, knowledge_base.problem_file)
        
        # Create a mapping from actions to integers
        self.action_mapping = {action: i for i, action in enumerate(self.action_space_generator.grounded_actions)}

        self.action_space = spaces.Discrete(len(self.action_space_generator.grounded_actions))
        self.observation_space = spaces.Box(low=-10, high=10, shape=(self.observation_generator.get_observation_size(),))

    def step(self, action):
        executed_info = self.execute_action(action)
        observation = self.observation_generator.get_observation()
        reward, done = self.reward_function_generator.get_reward(executed_info, action)
        info = {}
        return observation, reward, done, info

    def reset(self):
        self.observation_generator.reset()
        self.reward_function_generator.reset()
        self.action_space_generator.reset()
        return self.observation_generator.generate()

    def execute_action(self, action):
        action_name = self.action_mapping[action]
        
        # Assuming actions like "approach <object>", "grasp <object>", etc.
        action_type, target = action_name.split(" ")

        if action_type == "approach":
            success, info = self.call_service('execute_approach', target, Approach)
        elif action_type == "grasp":
            success, info = self.call_service('execute_grasp', target, Grasp)
        elif action_type == "place":
            success, info = self.call_service('execute_place', target, Place)

        return success, info

    def call_service(self, service_name, target, service_type):
        rospy.wait_for_service(service_name)

        try:
            execute_action = rospy.ServiceProxy(service_name, service_type)
            resp = execute_action(target)
            return resp.success, resp.info
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return False, str(e)

    def render(self, mode='human'):
        pass
