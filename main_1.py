import torch
from stable_baselines3 import PPO
use_pretrained_model = False
from env import Env
from utils import *
import pybullet as p
import time
from env import Env
# Initialize PyBullet in GUI mode
p.connect(p.GUI)

# Enable real-time simulation to continuously update the visualization
p.setRealTimeSimulation(True)

# Load your environment and other setup code
env = Env()

#
# # Load RL algorithm *************************************************************
if use_pretrained_model:
    model = PPO.load(f'./benchmarks/model_PPO.zip')
    model.set_env(env)
else:
    policy_kwargs = dict(activation_fn=torch.nn.ReLU,
                         net_arch=dict(pi=[l1_params, l2_params, l3_params],
                                       vf=[l1_params, l2_params, l3_params]))
    # add if need Tensorboard: tensorboard_log="./logs/"
    model = PPO("MlpPolicy", env, verbose=1, n_epochs=num_episodes)

# train the RL algorithm using SB3 Library **************************************
if do_training:
    model.learn(total_timesteps=simulation_steps)
    model.save(f'./benchmarks/model_PPO.zip')
#
# # test manually *****************************************************************
do_test = True
if do_test:
    # run the manual simulation loop
    env.run_simulation(model)

save_results(env.all_observations)

env.disconnect_server()
