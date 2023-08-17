import rospy
# from NovelGym import NovelGym

import gym
from gym import spaces

# Main modules
from ObservationGenerator import ObservationGenerator
from RewardFunction import RewardFunction
from ActionGenerator import ActionSpaceGenerator

# ROS related modules
from locobot_custom.srv import Approach, Grasp, Place

class RecycleBot(gym.Env):
    def __init__(self, knowledge_base):
        super(RecycleBot, self).__init__()
        print ("knowledge_base.domain_file: ", knowledge_base.domain_file)
        print ("knowledge_base.problem_file: ", knowledge_base.problem_file)
        print ("knowledge_base.failed_action: ", knowledge_base.failed_action)


        self.observation_generator = ObservationGenerator(knowledge_base.domain_file, knowledge_base.problem_file)
        self.reward_function_generator = RewardFunction(knowledge_base.domain_file, knowledge_base.problem_file, knowledge_base.failed_action)
        self.action_space_generator = ActionSpaceGenerator(knowledge_base.domain_file, knowledge_base.problem_file)
        
        # Create a mapping from actions to integers
        self.action_mapping = {i: action for i, action in enumerate(self.action_space_generator.generate_grounded_actions())}
        print ("self.action_mapping: ", self.action_mapping)

        self.action_space = spaces.Discrete(len(self.action_space_generator.grounded_actions))
        self.observation_space = spaces.Box(low=-10, high=10, shape=(self.observation_generator.get_observation_space_size(),))

    def step(self, action):
        executed_info = self.execute_action(action)
        observation = self.observation_generator.get_observation()
        reward, done = self.reward_function_generator.get_reward(executed_info, action)
        info = {}
        return (observation, reward, done, info)

    def reset(self):
        self.observation_generator.reset()
        # self.reward_function_generator.reset()
        # self.action_space_generator.reset()
        return self.observation_generator.get_observation()

    def execute_action(self, action):   
        print ("action: ", action)
        action_name = self.action_mapping[action]
        print ("action_name: ", action_name)
        # this is hardcoded for now.
        action_type = action_name.split(" ")[0]
        target = action_name.split(" ")[1]
        # Assuming actions like "approach <object>", "grasp <object>", etc.
        # action_type, target = action_name.split(" ")

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

def main():
    # Initialize ROS node (if not already initialized)
    if not rospy.get_node_uri():
        rospy.init_node('recycle_bot_env_test', anonymous=True)
    class KnowledgeBase:
        def __init__(self):
            self.domain_file = "../knowledge/PDDL/recycle_bot/domain.pddl"
            self.problem_file = "../knowledge/PDDL/recycle_bot/problem.pddl"   
            self.failed_action = "(approach doorway_1 room_1 ball_1)"

    knowledge_base = KnowledgeBase()

    # Create an environment instance
    env = RecycleBot(knowledge_base)

    # Number of episodes
    num_episodes = 10

    for episode in range(num_episodes):
        observation = env.reset()
        done = False
        episode_reward = 0

        while not done:
            # For simplicity, we're just sampling random actions. You can replace this with a policy or other logic.
            action = env.action_space.sample()
            observation, reward, done, info = env.step(action)
            episode_reward += reward

        print(f"Episode {episode + 1} finished with reward: {episode_reward}")

if __name__ == "__main__":
    main()
