from abc import ABC

import numpy as np
import pybullet as p
from kuka import Kuka
from pybullet_utils import urdfEditor as ed

from config import *


class CustomKuka(Kuka, ABC):
    def __init__(self):
        """
        A class that contains all attributes and methods related to the Kuka Robot
        It inherits from Kuka class of Pybullet library
        """

        # by initializing the super class, the robot is ready to work
        super().__init__()

        # logging  *****************************************************************
        self.robot_positions = np.zeros((simulation_steps, 3))
        self.robot_orientations = np.zeros((simulation_steps, 4))

        # in each simulation step, for each joint:
        self.joints_positions = np.empty((simulation_steps, self.numJoints, 1))
        self.joints_velocity = np.empty((simulation_steps, self.numJoints, 1))

    def log_all_states(self, step):
        position_and_orientation = p.getBasePositionAndOrientation(self.kukaUid)
        self.robot_positions[step, :] = np.array(position_and_orientation[0])
        self.robot_orientations[step, :] = np.array(position_and_orientation[1])

        for j in range(self.numJoints):
            joint_state = p.getJointState(self.kukaUid, j)
            self.joints_positions[step, j, :] = np.array([joint_state[0]])
            self.joints_velocity[step, j, :] = np.array([joint_state[1]])
