class Learner(object):

    def __init__(self, name: str,
                       as_size: int, 
                       obs_size: int) -> None: 

        self.name = name
        self.as_size = as_size
        self.obs_size = obs_size

    def run_episode(self, max_time_steps: int):
        raise NotImplementedError

    def get_action(self, observation: list):
        raise NotImplementedError
    
    def give_reward(self, reward: int):
        raise NotImplementedError

    def update_weights(self):
        raise NotImplementedError
    
    def load_weights(self):
        raise NotImplementedError
    
    def save_weights(self):
        raise NotImplementedError
    
