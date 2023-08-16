Observation = list
Reward = float
Done = bool
Info = str

class NovelGym():
    def __init__():
        pass

    def step(self, action):
        raise NotImplementedError

    def reset(self) -> tuple(Observation, Reward, Done, Info): 
        raise NotImplementedError 
