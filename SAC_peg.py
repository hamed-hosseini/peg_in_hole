import gymnasium as gym
from config import *
from utils import *
from stable_baselines3 import SAC
import pybullet as p
import time
from env import Env
import torch
# Initialize PyBullet in GUI mode
p.connect(p.GUI)

# Enable real-time simulation to continuously update the visualization
p.setRealTimeSimulation(True)
# Set the default data type for all floating-point tensors to float32
# torch.set_default_dtype(torch.float32)

# Load your environment and other setup code
env = Env()
env.reset()
model = SAC("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=totoll_simulation_steps, log_interval=4)
model.save("peghole")

# del model # remove to demonstrate saving and loading
#
model = SAC.load("peghole")
print('hamed')
obs = env.reset()
# while True:
#     action, _states = model.predict(obs, deterministic=True)
#     obs, reward,  terminated, info = env.step(action)
#     if terminated:
#         obs = env.reset()
if do_test:
    # run the manual simulation loop
    env.run_simulation(model)

save_results(env.all_observations)

env.disconnect_server()