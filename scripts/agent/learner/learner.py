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
        """
        Given the info of a run, update the replay buffer.
        At the same time, Run an update.
        Transition dict should be a dict of items in the following format:
        {
            "obs": *,        # np.array, the observation before the transition
            "obs_next": *,   # np.array, the observation after the transition
            "done": *,       # bool, terminated or truncated, see below
            "act": *,        # int, the action taken at this time step
            "rew": *,        # double, the reward given at this time step
            "terminated": *  # bool, whether the game reached a terminal state
            "truncated": *   # bool, whether the game reached max episode
                             #        and have to be truncated
        }
        """
        raise NotImplementedError
