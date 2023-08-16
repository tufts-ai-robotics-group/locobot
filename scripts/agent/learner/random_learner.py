import gymnasium as gym

class Learner:
    """
    A Baseline random learner that randomly samples actions
    """
    def __init__(self, obs_space: gym.Space, act_space: gym.Space) -> None: 
        self.obs_space = obs_space
        self.act_space = act_space

    def get_action(self, obs):
        return self.act_space.sample()
    
    def update(self, obs):
        pass
