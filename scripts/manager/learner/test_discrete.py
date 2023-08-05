from ppo_learner import PPOLearner
import gymnasium as gym

def test():
    env = gym.make("CartPole-v1", max_episode_steps=200, render_mode="human")
    learner = PPOLearner(
        obs_space=env.observation_space,
        act_space=env.action_space,
        hidden_sizes=[256]
    )

    env.reset(seed=0)

    for epi in range(500):
        observation, info = env.reset()
        terminated = False
        truncated = False
        epi_len = 0
        total_return = 0

        while not terminated and not truncated:
            action = learner.get_action(observation)
            prev_obs = observation
            observation, reward, terminated, truncated, info = env.step(action)

            trans_dict = {
                "obs": prev_obs,
                "obs_next": observation,
                "rew": reward,
                "act": action,
                "done": terminated or truncated,
                "truncated": truncated,
                "terminated": terminated,
                "info": info
            }
            
            epi_len += 1
            total_return += reward

            learner.update(trans_dict)
        print(f"epi: {epi}, return: {total_return}")


if __name__ == "__main__":
    test()
