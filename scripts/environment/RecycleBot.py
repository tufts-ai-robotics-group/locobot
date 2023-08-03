import gym
from gym import spaces

# Main modules
from ObservationGenerator import ObservationGenerator
from RewardFunction import RewardFunction
from ActionGenerator import ActionSpaceGenerator

class RecycleBot(gym.Env):
    def __init__(self, knowledge_base):
        super(RecycleBot, self).__init__()
        # TODO: The problem file should be generated dynamically. So the domain file should be enough.
        # We need to therefore not pass problem file as an argument for the __init__ function.
        # Once initialized, the environment should be able to generate the problem file dynamically.
        # Modify the entire code base accordingly. Very important.

        self.observation_generator = ObservationGenerator(knowledge_base.domain_file, knowledge_base.problem_file)
        self.reward_function_generator = RewardFunction(knowledge_base.domain_file, knowledge_base.problem_file, knowledge_base.failed_action)
        self.action_space_generator = ActionSpaceGenerator(knowledge_base.domain_file, knowledge_base.problem_file)
        # Create a mapping from actions to integers
        self.action_mapping = {action: i for i, action in enumerate(self.action_space_generator.grounded_actions)}

        self.action_space = spaces.Discrete(len(self.action_space_generator.grounded_actions))
        self.observation_space = spaces.Box(low=-10, high=10, shape=(self.observation_generator.get_observation_size(),))

    def step(self, action):
        # Execute one time step within the environment
        executed_info = self.execute_action(action)
        # we need to return the infor =rmation aboit the action executed
        # it can be executed succesfully, or it can fail and failure can be due to many reasons
        # we need to return the information about the action executed
        # One reeason for failure can be that the action is not applicable in the current state
        # insufficient pre conditions can be another reason
        # can be a totally impossible action.
        observation = self.observation_generator.get_observation()
        reward, done = self.reward_function_generator.get_reward(executed_info, action)
        info = {}
        return observation, reward, done, info

    def reset(self):
        # Reset the state of the environment to an initial state
        self.observation_generator.reset()
        self.reward_function_generator.reset()
        self.action_space_generator.reset()
        return self.observation_generator.generate()

    def execute_action(self, action):
        # Execute the action in the environment
        # This function should return the information about the action executed
        # It can be executed succesfully, or it can fail and failure can be due to many reasons
        # We need to return the information about the action executed
        # One reeason for failure can be that the action is not applicable in the current state
        # insufficient pre conditions can be another reason
        # can be a totally impossible action.
        pass

    def render(self, mode='human'):
        # Render the environment to the screen
        pass
