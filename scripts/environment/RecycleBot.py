# In recycle_bot_env.py
from ObservationGenerator import ObservationGenerator
from RewardFunction import RewardFunction

class RecycleBotEnv:
    def __init__(self, domain_file, problem_file, failed_action):
        # Initialize state and action spaces
        self.state = None # need to implement it
        self.action_space = None # need to implement it
        self.observation_generator = ObservationGenerator(domain_file, problem_file)
        self.reward_function = RewardFunction(domain_file, problem_file, failed_action)
        

    def reset(self):
        # Reset environment to initial state
        raise NotImplementedError

    def step(self, action):
        # Execute action and return new state, reward, done, info
        new_state = self.execute_action(action)
        reward = self.reward_function.get_reward(action)
        done = self.is_done()
        info = None
        return new_state, reward, done, info
    
        raise NotImplementedError

    def render(self):
        # Render current state of the environment (not implemented)
        raise NotImplementedError

    def close(self):
        # Clean up resources used by the environment
        pass

    def seed(self, seed=None):
        # Set the seed for this env's random number generator(s)
        pass

    def get_state(self):
        # Return current state of the environment
        return self.state
    
