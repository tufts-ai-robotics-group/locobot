import gymnasium as gym
from torch import nn

class Learner:
    """
    Base Abstract Class for Learner
    """
    def __init__(self, obs_space: gym.Space, act_space: gym.Space) -> None: 
        self.obs_space = obs_space
        self.act_space = act_space
        self.transition_hist = {
            "action": [],
            "reward": [],
            "state": [],
            "next_state": [],
            "truncated": [],
            "terminated": []
        }

    def get_action(self, obs):
        raise NotImplementedError
    
    def update(self, transition_dict):
        raise NotImplementedError
