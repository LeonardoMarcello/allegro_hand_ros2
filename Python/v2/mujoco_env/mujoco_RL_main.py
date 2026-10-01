import gymnasium
import mujoco_env

from gymnasium.envs.registration import register

# Register and create environment
register(
    id='mujoco_env/MyEnv-v0',
    entry_point='mujoco_env.envs.my_env:MyEnv',  # <-- YOU NEED TO CHANGE THIS LINE
    #max_episode_steps=1000,                # Optional: specify when the episode truncates
)
env = gymnasium.make('mujoco_env/MyEnv-v0', render_mode='human')

# Reset the environment with a specific seed for reproducibility
observation, info = env.reset(seed=42)

# Run simulation for a fixed number of steps
# for _ in range(1000):
while True:
    # Choose a random action from the available action space
    action = env.action_space.sample()

    # Take a step in the environment using the chosen action
    observation, reward, terminated, truncated, info = env.step(action)

    # Check if the episode is over (terminated) or max steps reached (truncated)
    if terminated or truncated:
        # If the episode ends or is truncated, reset the environment
        observation, info = env.reset()

# Close the environment when the simulation is done
env.close()