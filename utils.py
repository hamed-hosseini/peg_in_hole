import math

import pandas as pd
import pybullet as p
import pybullet_data
from matplotlib import pyplot as plt

from config import *


def create_objects():
    # add the path to import objects in the Pybullet library
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # set gravity for the environment
    p.setGravity(*gravity_values)

    # Create Objects  *****************************************************************

    # create the plane
    plane_id = p.loadURDF('plane.urdf', basePosition=plane_base_position, useFixedBase=True)

    # create the table with custom dynamics
    table_id = p.loadURDF('table/table.urdf', basePosition=table_base_position, useFixedBase=True)

    # create double hole
    hole_id = p.loadURDF('objects/hole.urdf',  # objects/multi_hole_v1
                         basePosition=hole_base_position,
                         baseOrientation=p.getQuaternionFromEuler([0, 0, 1.5 * math.pi]),
                         flags=0,
                         globalScaling=0.001,
                         useFixedBase=True)
    p.changeDynamics(hole_id, linkIndex=-1, mass=10, contactStiffness=100000.0, contactDamping=1.0)
    hole_position, hole_orientation = p.getBasePositionAndOrientation(hole_id)
    table_position, table_orientation = p.getBasePositionAndOrientation(table_id)

    # Calculate the relative transformation between hole and table
    relative_position = [hole_position[i] - table_position[i] for i in range(3)]
    relative_orientation = p.getDifferenceQuaternion(hole_orientation, table_orientation)

    # Create a fixed constraint to make the hole solidly fixed on the table
    constraint_id = p.createConstraint(
        parentBodyUniqueId=table_id,
        parentLinkIndex=-1,
        childBodyUniqueId=hole_id,
        childLinkIndex=-1,
        jointType=p.JOINT_FIXED,
        jointAxis=[0, 0, 0],
        parentFramePosition=relative_position,
        childFramePosition=[0, 0, 0],
        parentFrameOrientation=table_orientation,
        childFrameOrientation=relative_orientation
    )

    return [plane_id, table_id, hole_id]


def getExtendedObservation(observation, gripperState, kuka_id):
    # taken from:
    # github.com/bulletphysics/bullet3/blob/master/examples/pybullet/gym/pybullet_envs/bullet/kukaGymEnv.py#L107
    # merge observation from the robot and Gripper

    gripperPos = gripperState[0]
    gripperOrn = gripperState[1]
    blockPos, blockOrn = p.getBasePositionAndOrientation(0)

    invGripperPos, invGripperOrn = p.invertTransform(gripperPos, gripperOrn)
    gripperMat = p.getMatrixFromQuaternion(gripperOrn)
    dir0 = [gripperMat[0], gripperMat[3], gripperMat[6]]
    dir1 = [gripperMat[1], gripperMat[4], gripperMat[7]]
    dir2 = [gripperMat[2], gripperMat[5], gripperMat[8]]

    gripperEul = p.getEulerFromQuaternion(gripperOrn)
    # print("gripperEul")
    # print(gripperEul)
    blockPosInGripper, blockOrnInGripper = p.multiplyTransforms(invGripperPos, invGripperOrn,
                                                                blockPos, blockOrn)
    projectedBlockPos2D = [blockPosInGripper[0], blockPosInGripper[1]]
    blockEulerInGripper = p.getEulerFromQuaternion(blockOrnInGripper)
    # print("projectedBlockPos2D")
    # print(projectedBlockPos2D)
    # print("blockEulerInGripper")
    # print(blockEulerInGripper)

    # we return the relative x,y position and euler angle of block in gripper space
    blockInGripperPosXYEulZ = [blockPosInGripper[0], blockPosInGripper[1], blockEulerInGripper[2]]

    # observation.extend(list(blockInGripperPosXYEulZ))
    # return observation
    return observation


def save_results(all_observations):
    p.saveWorld('results/world')

    # convert observation to Pandas DataFrame, which is more suitable for plotting
    all_observations_df = pd.DataFrame(all_observations)

    # Actions, Voltage and Current Per episode
    fig, axes = plt.subplots(nrows=2, figsize=(10, 10))
    fig.suptitle(f'All observations of the algorithm', fontsize=20)

    all_observations_df.iloc[:, 0:3].plot(ax=axes[0], title='Positions')
    axes[0].legend(['x', 'y', 'z'])
    # all_observations_df.iloc[:, 3:6].plot(ax=axes[1], title='Orientations')
    all_observations_df.iloc[:, 3:6].plot(ax=axes[1], title='Forces')
    axes[1].legend(['fx', 'fy', 'fz'])
    # all_observations_df.iloc[:, 12:15].plot(ax=axes[3], title='Gripper')

    plt.subplots_adjust(right=0.9)
    fig.savefig(f'results/all_observations.jpg')
