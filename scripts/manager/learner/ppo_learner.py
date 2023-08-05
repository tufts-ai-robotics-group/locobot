import gymnasium as gym
from torch import nn
import torch
import numpy as np

import tianshou as ts
from tianshou.utils.net.common import Net, ActorCritic, MLP

class PPOLearner:
    """
    A wrapper for an PPO learner provided by tianshou
    """
    def __init__(self, 
                 obs_space: gym.Space, 
                 act_space: gym.Space, 
                 hidden_sizes=[256], 
                 lr=5e-3,
                 sample_size=64,
                 replay_buffer_size=10000,
                 device="cpu"
    ) -> None: 
        self.obs_space = obs_space
        self.act_space = act_space
        self.replay_buffer = ts.data.ReplayBuffer(replay_buffer_size)
        self.sample_size = sample_size

        self.device = device
        self.actor_net = Net(np.prod(obs_space.shape), act_space.n, softmax=True, device=device)
        self.critic_net = MLP(np.prod(obs_space.shape), 1, hidden_sizes=hidden_sizes, device=device)
        actor_critic = ActorCritic(self.actor_net, self.critic_net)
        self.optim = torch.optim.Adam(actor_critic.parameters(), lr=lr)
        self.policy = ts.policy.PPOPolicy(
            actor=self.actor_net,
            critic=self.critic_net,
            optim=self.optim,
            dist_fn=torch.distributions.Categorical
        ).to(device)

    def get_action(self, obs):
        act = self.policy.forward(ts.data.Batch(obs=[obs], info=None)).act
        return act.numpy()[0]
    
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
        transition_dict['obs'] = torch.tensor(transition_dict['obs'], device=self.device)
        transition_dict['obs_next'] = torch.tensor(transition_dict['obs_next'], device=self.device)
        batch = ts.data.Batch(transition_dict)
        result = self.replay_buffer.add(batch)
        # print(transition_dict)
        if len(self.replay_buffer) >= 2 * self.sample_size:
            self.policy.update(
                self.sample_size, self.replay_buffer,
                batch_size=64, repeat=3
            )
