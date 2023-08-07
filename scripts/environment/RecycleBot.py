import gym
from gym import spaces

# Main modules
from ObservationGenerator import ObservationGenerator
from RewardFunction import RewardFunction
from ActionGenerator import ActionSpaceGenerator

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
        action_type, target = action_name.split(" ")

        # Here, we don't call any ROS functions directly.
        # Instead, we communicate with the bridge node which will handle the ROS interaction.
        # This can be done using any IPC method - for simplicity, let's assume a shared variable, 
        # but you'd typically use something more robust like a socket, named pipe, etc.
        
        shared_variable = {"action_type": action_type, "target": target}
        
        # You'd then wait for the bridge node to update this shared state with the result of the action.
        # (This is a simplification; in practice, you might use some form of signaling or event system.)
        while not "result" in shared_variable:
            pass  # Waiting for result to be populated by bridge node

        executed_info = shared_variable["result"]
        return executed_info

    def render(self, mode='human'):
        pass
