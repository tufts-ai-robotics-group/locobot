import rospy
# from NovelGym import NovelGym

import gym
from gym import spaces

# Main modules
from ObservationGenerator import ObservationGenerator
from RewardFunction import RewardFunction
from ActionGenerator import ActionSpaceGenerator

# ROS related modules
from locobot_custom.srv import Approach, Grasp, Place, PrimitiveBase


class RecycleBot(gym.Env):
    def __init__(self, domain, problem, failed_action, planner, sym_actions_dict, executor_dir, num_eps=100):
        super(RecycleBot, self).__init__()

        self.observation_generator = ObservationGenerator(domain, problem)
        self.reward_function_generator = RewardFunction(domain, problem, failed_action)
        self.action_space_generator = ActionSpaceGenerator(domain, problem)
        
        # Create a mapping from actions to integers
        self.action_mapping = {i: action for i, action in enumerate(self.action_space_generator.generate_grounded_actions())}
        print ("self.action_mapping: ", self.action_mapping)

        self.action_space = spaces.Discrete(len(self.action_space_generator.grounded_actions))

        # Initialize RecycleBotAgent
        self._planner = planner
        self._sym_actions_dict = sym_actions_dict
        self._executor_dir = executor_dir
        self._num_eps = num_eps
        self.problem_file_path = problem

        from RecycleBotAgent import RecycleBotAgent
        self.agent = RecycleBotAgent(self._planner, self._sym_actions_dict, self._executor_dir, self._num_eps)
        self.observation_space = spaces.Box(low=-10, high=10, shape=(self.observation_generator.get_observation_space_size(),))

    def step(self, action):
        # TODO:
        # we need to verify if the action is valid or not.
        # by checking the pre conditions  # and then execute the action.

        success, executed_info = self.execute_action(action)
        # print ("success: ", success)
        # print ("executed_info: ", executed_info)

        observation = self.observation_generator.get_observation()
        # Call new_problem() of the agent after getting new observation
        self.agent.new_problem()
        # print ("observation after taking a step in the world: ", observation)
        reward, terminated, truncated = self.reward_function_generator.get_reward(success, executed_info, problem_file=self.problem_file_path)
        info = executed_info
        return (observation, reward, terminated, truncated, info)

    def reset(self):
        self.observation_generator.reset()
        # self.reward_function_generator.reset()
        # self.action_space_generator.reset()
        # print ("self.observation_generator.get_observation(): ", self.observation_generator.get_observation())
        return self.observation_generator.get_observation(), {}

    def execute_action(self, action):   
        print ("action: ", action)
        action_name = self.action_mapping[action]
        print ("action_name: ", action_name)
        # this is hardcoded for now.
        action_type = action_name.split(" ")[0]
        target = action_name.split(" ")[1] # in our case, the target of the action is the first parameter of the action.
        success = False
        info = ""

        if action_type == "approach":
            success, info = self.call_service('approach', target, Approach)
        elif action_type == "grasp":
            success, info = self.call_service('grasp', target, Grasp)
        elif action_type == "place":
            success, info = self.call_service('place', target, Place)
        elif action_type == "pass_through_door":
            success, info = self.call_service('approach', "doorway_1_blue", Approach)
        # primitive actions
        elif action_type == "move_forward":
            success, info = self.call_service('primitive_action_service', {"action_type": "move_forward", "value": 1.0}, PrimitiveBase)
        elif action_type == "turn_left":
            success, info = self.call_service('primitive_action_service', {"action_type": "turn_left", "value": 0.785398}, PrimitiveBase)
        elif action_type == "turn_right":
            success, info = self.call_service('primitive_action_service', {"action_type": "turn_right", "value": 0.785398}, PrimitiveBase)

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