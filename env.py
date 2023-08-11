import time
from abc import ABC

import gym
import numpy as np

from custom_kuka import CustomKuka
from utils import *
from tqdm import tqdm  # Import tqdm for the progress bar
from config import totoll_simulation_steps

class Env(gym.Env, ABC):
    def __init__(self):
        # General Setups  ************************************************************

        # execute the Pybullet Server
        # p.connect(p.GUI) if is_gui_active else p.connect(p.DIRECT)
        p.resetSimulation()

        # enable or disable real-time simulation
        # if zero, use stepSimulation method inside the simulation loop
        p.setRealTimeSimulation(0)

        # to configure some settings of the built-in OpenGL visualizer
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)

        self.plane_id, self.table_id, self.hole_id, = create_objects()

        # create the robot
        self.kuka = CustomKuka()

        # RL setups  *****************************************************************

        # A box in the gym library is a continuous multidimensional space
        # we have a vector of size 6
        self.action_space = gym.spaces.Box(low=-0.1, high=+0.1,
                                           shape=(action_vector_size,), dtype=float)

        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf,
                                                shape=(obs_vector_size,), dtype=float)

        self.rl_counter = 0

        # logging  *******************************************************************
        self.all_observations = np.empty((simulation_steps, obs_vector_size))
        self.progress_bar = tqdm(total=totoll_simulation_steps, desc="Training Progress")

    def run_simulation(self, model=True):
        start_time = time.time()

        observation = getExtendedObservation(observation=self.kuka.getObservation(),
                                             gripperState=p.getLinkState(self.kuka.kukaUid,
                                                                         self.kuka.kukaGripperIndex),
                                             kuka_id=self.kuka.kukaUid)

        for step in range(simulation_steps):
            # action for all joints
            # action = self.action_space.sample()
            action = model.predict(observation)[0]
            # action = [-1.0, -1.0, -20.0, -11, -1.0, -2.0, -1.0, 1.5]
            # apply motor control to joints and get the observation and reward
            observation, reward, done, info = self.step(action)
            if done:
                print('ending test episode')
                self.reset()
            self.all_observations[step, :] = np.array(observation)

            # log everything that is useful for analysis and debugging
            # self.kuka.log_all_states(step)

        time.sleep(time_resolution_per_second)
        print(f'elapsed time is {time.time() - start_time}')
        # print(f'positions are {self.kuka.robot_positions}')

    def step(self, action):
        # print('step')
        # converting actions to real actions according to the Pybullet library
        # truncated = False
        dv = 0.005
        dx = np.array(action)[0] * dv
        dy = np.array(action)[1] * dv
        dz = np.array(action)[2] * dv
        # dw1 = np.array(action)[3]
        # dw2 = np.array(action)[4]
        # dw3 = np.array(action)[5]

        # da = np.array(action)[2] * 0.05
        # f = 0.3
        # realAction = [dx, dy, 0.002, da, f]
        # realAction = [0, 0, -0.00002, 0, 0]
        realAction = [dx, dy, -0.0002]
        # realAction = [0, 0, -.00002]

        self.kuka.applyAction(realAction)

        p.stepSimulation()

        # output of the sensor
        observation = getExtendedObservation(observation=self.kuka.getObservation(),
                                             gripperState=p.getLinkState(self.kuka.kukaUid,
                                                                         self.kuka.kukaGripperIndex),
                                             kuka_id=self.kuka.kukaUid)
        # reward
        # peg_position = np.array(p.getLinkState(self.kuka.kukaUid, self.kuka.kukaGripperIndex)[0])
        # gripper_position = np.array(p.getBasePositionAndOrientation(self.kuka.kukaUid)[0])
        # distance = np.linalg.norm(peg_position - gripper_position)
        # reward = - distance

        peg_position = observation[:3]
        distance_z = (peg_position[2] - hole_base_position[2])
        distance_xy = np.linalg.norm(np.array(peg_position[:2]) - np.array(hole_base_position[:2]))
        # reward = - distance_z /0.5 - distance_xy/0.5 - np.linalg.norm(observation[3:])/40.0
        reward = - distance_z /0.5 - distance_xy/0.5
        print(f'distance_z{distance_z}', f'distance_xy{distance_xy}')
        # whether the episode has finished or not
        self.rl_counter += 1
        # print(np.linalg.norm(np.array(peg_position) - np.array(list(hole_base_position))))
        # print(np.array(peg_position), np.array(list(hole_base_position)))
        done1 = int(self.rl_counter >= simulation_steps) or (np.linalg.norm(np.array(peg_position) - np.array(list(hole_base_position))) < 0.03)
        done2 = (peg_position[2] < hole_head_position[2] and distance_xy > 0.02)
        # if (np.linalg.norm(np.array(peg_position) - np.array(list(hole_base_position))) < 0.03):
        #     print('reached')
        if done1:
            r_end = (int((np.linalg.norm(np.array(peg_position) - np.array(list(hole_base_position))) < 0.03)) - 0.5) * 2
            print('done', r_end)
            reward = reward + r_end
        if done2:
            reward = reward - 1.0

        # extra info
        info = {}
        # if observation[3] > 40 or observation[4] > 40 or observation[5] > 40:
        #     truncated = True
        done = done1 or done2
        self.progress_bar.update(1)
        return observation, reward, done, info

    @staticmethod
    def disconnect_server():
        p.disconnect()

    def reset(self, *args):
        print('reset')
        p.resetSimulation()

        # enable or disable real-time simulation
        # if zero, use stepSimulation method inside the simulation loop
        p.setRealTimeSimulation(0)

        # to configure some settings of the built-in OpenGL visualizer
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)

        # self.plane_id, self.table_id, self.peg_id, self.hole_id, = create_objects()
        self.plane_id, self.table_id, self.hole_id, = create_objects()

        # create the robot
        self.kuka = CustomKuka()

        self.rl_counter = 0

        observation = getExtendedObservation(observation=self.kuka.getObservation(),
                                             gripperState=p.getLinkState(self.kuka.kukaUid,
                                                                         self.kuka.kukaGripperIndex),
                                             kuka_id=self.kuka.kukaUid)
        return observation
