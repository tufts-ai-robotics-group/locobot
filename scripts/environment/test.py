import gymnasium
from stable_baselines3 import PPO

# Create the environment
env = gymnasium.make('RecycleBot-v0')

# Initialize the agent
model = PPO('MlpPolicy', env, verbose=1)

# Train the agent
model.learn(total_timesteps=10000)

# Save the trained agent
model.save('ppo_recyclebot')

# Later, to load the trained agent:
# model = PPO.load('ppo_recyclebot')

# You can use the trained model to make predictions
# obs = env.reset()
# action, _states = model.predict(obs)
