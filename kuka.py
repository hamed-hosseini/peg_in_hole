import inspect
import math
import os

import pybullet as p
import pybullet_data

from config import *

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)


class Kuka:
    def __init__(self, urdfRootPath=pybullet_data.getDataPath(), timeStep=0.01):
        self.urdfRootPath = urdfRootPath
        self.timeStep = timeStep
        self.maxVelocity = .35
        self.maxForce = 200.
        self.fingerAForce = 2
        self.fingerBForce = 2.5
        self.fingerTipForce = 2
        self.useInverseKinematics = 1
        self.useSimulation = 1
        self.useNullSpace = 1
        self.useOrientation = 1
        self.kukaEndEffectorIndex = 6
        self.kukaGripperIndex = 7
        # lower limits for null space
        self.ll = [-.967, -2, -2.96, 0.19, -2.96, -2.09, -3.05]
        # upper limits for null space
        self.ul = [.967, 2, 2.96, 2.29, 2.96, 2.09, 3.05]
        # joint ranges for null space
        self.jr = [5.8, 4, 5.8, 4, 5.8, 4, 6]
        # rest-poses for null space
        self.rp = [0, 0, 0, 0.5 * math.pi, 0, -math.pi * 0.5 * 0.66, 0]

        # joint damping coefficients
        self.jd = [
            0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001,
            0.00001, 0.00001, 0.00001, 0.00001
        ]
        self.reset()

    def reset(self):
        # "kuka_iiwa/kuka_with_gripper2.sdf"
        self.kukaUid = p.loadSDF("objects/robot_peg.sdf")[0]

        # for i in range (p.getNumJoints(self.kukaUid)):
        #  print(p.getJointInfo(self.kukaUid,i))
        p.resetBasePositionAndOrientation(self.kukaUid, robot_base_position,
                                          [0.000000, 0.000000, 0.000000, 1.000000])

        # self.jointPositions = [
        #     0.006418, 0.413184, -0.011401, -1.589317, 0.005379, 1.137684, -0.006539, 0.000048,
        #     -0.299912, 0.000000, -0.000043, 0.299960, 0.000000, -0.000200
        # ]
        self.jointPositions = [
            math.pi, 0, 0, 0.5 * math.pi, 0, -0.25 * math.pi, 0, 0.000048,
            -0.299912, 0.000000, -0.000043, 0.299960, 0.000000, -0.000200
        ]

        self.numJoints = p.getNumJoints(self.kukaUid)
        for jointIndex in range(self.numJoints):
            p.resetJointState(self.kukaUid, jointIndex, self.jointPositions[jointIndex])

            # to enable sensor on the joint
            p.enableJointForceTorqueSensor(self.kukaUid, jointIndex, enableSensor=True)

            p.setJointMotorControl2(self.kukaUid,
                                    jointIndex,
                                    p.POSITION_CONTROL,
                                    targetPosition=self.jointPositions[jointIndex],
                                    force=self.maxForce)

        # self.endEffectorPos = [0.537, 0.0, 0.5]
        # self.endEffectorPos = [-1, 0.0, 1.5]
        self.endEffectorPos = [0.5, 0, 1.5]
        # self.endEffectorPos = [-3, 6, 2]
        self.endEffectorAngle = 0

        self.motorNames = []
        self.motorIndices = []

        for i in range(self.numJoints):
            jointInfo = p.getJointInfo(self.kukaUid, i)
            qIndex = jointInfo[3]
            if qIndex > -1:
                # print("motorname")
                # print(jointInfo[1])
                self.motorNames.append(str(jointInfo[1]))
                self.motorIndices.append(i)

    def getActionDimension(self):
        if (self.useInverseKinematics):
            return len(self.motorIndices)
        return 6  # position x,y,z and roll/pitch/yaw euler angles of end effector

    def getObservationDimension(self):
        return len(self.getObservation())

    def getObservation(self):
        observation = []
        state = p.getLinkState(self.kukaUid, self.kukaGripperIndex)
        pos = state[0]
        # print(pos)
        orn = state[1]
        euler = p.getEulerFromQuaternion(orn)

        forces = p.getJointState(self.kukaUid, self.kukaGripperIndex)[2][:3]

        observation.extend(list(pos))
        # observation.extend(list(euler))
        observation.extend(list(forces))
        # print(p.getJointState(self.kukaUid, self.kukaGripperIndex)[2])

        return observation

    def applyAction(self, motorCommands):

        # print ("self.numJoints")
        # print (self.numJoints)
        if (self.useInverseKinematics):

            dx = motorCommands[0]
            dy = motorCommands[1]
            dz = motorCommands[2]
            # da = motorCommands[3]
            # fingerAngle = motorCommands[4]

            state = p.getLinkState(self.kukaUid, self.kukaEndEffectorIndex)
            actualEndEffectorPos = state[0]
            # print("pos[2] (getLinkState(kukaEndEffectorIndex)")
            # print(actualEndEffectorPos[2])

            self.endEffectorPos[0] = self.endEffectorPos[0] + dx
            if (self.endEffectorPos[0] > hole_base_position[0] + 0.05):
                self.endEffectorPos[0] = hole_base_position[0] + 0.05
            if (self.endEffectorPos[0] < hole_base_position[0] - 0.05):
                self.endEffectorPos[0] = hole_base_position[0] - 0.05
            self.endEffectorPos[1] = self.endEffectorPos[1] + dy
            if (self.endEffectorPos[1] < hole_base_position[1] - 0.05):
                self.endEffectorPos[1] = hole_base_position[1] - 0.05
            if (self.endEffectorPos[1] > hole_base_position[1] + 0.05):
                self.endEffectorPos[1] = hole_base_position[1] + 0.05
            self.endEffectorPos[2] = self.endEffectorPos[2] + dz
            if (self.endEffectorPos[2] < 0.8):
                self.endEffectorPos[2] = 0.8
            if (self.endEffectorPos[2] > 1.5):
                self.endEffectorPos[2] = 1.5
            # print ("self.endEffectorPos[2]")
            # print (self.endEffectorPos[2])
            # print("actualEndEffectorPos[2]")
            # print(actualEndEffectorPos[2])
            # if (dz<0 or actualEndEffectorPos[2]<0.5):
            #     self.endEffectorPos[2] = self.endEffectorPos[2] + dz
            #
            # self.endEffectorAngle = self.endEffectorAngle + da
            pos = self.endEffectorPos
            orn = p.getQuaternionFromEuler([0, -math.pi, 0])  # -math.pi,yaw])
            # orn = p.getQuaternionFromEuler([dw1, dw2, dw3])  # -math.pi,yaw])
            if (self.useNullSpace == 1):
                if (self.useOrientation == 1):
                    jointPoses = p.calculateInverseKinematics(self.kukaUid, self.kukaEndEffectorIndex, pos,
                                                              orn, self.ll, self.ul, self.jr, self.rp)
                else:
                    jointPoses = p.calculateInverseKinematics(self.kukaUid,
                                                              self.kukaEndEffectorIndex,
                                                              pos,
                                                              lowerLimits=self.ll,
                                                              upperLimits=self.ul,
                                                              jointRanges=self.jr,
                                                              restPoses=self.rp)
            else:
                if (self.useOrientation == 1):
                    jointPoses = p.calculateInverseKinematics(self.kukaUid,
                                                              self.kukaEndEffectorIndex,
                                                              pos,
                                                              orn,
                                                              jointDamping=self.jd)
                else:
                    jointPoses = p.calculateInverseKinematics(self.kukaUid, self.kukaEndEffectorIndex, pos)

            # print("jointPoses")
            # print(jointPoses)
            # print("self.kukaEndEffectorIndex")
            # print(self.kukaEndEffectorIndex)
            if (self.useSimulation):
                for i in range(self.kukaEndEffectorIndex + 1):
                    # print(i)
                    p.setJointMotorControl2(bodyUniqueId=self.kukaUid,
                                            jointIndex=i,
                                            controlMode=p.POSITION_CONTROL,
                                            targetPosition=jointPoses[i],
                                            targetVelocity=0,
                                            force=self.maxForce,
                                            maxVelocity=self.maxVelocity,
                                            positionGain=0.3,
                                            velocityGain=1)
            else:
                # reset the joint state (ignoring all dynamics, not recommended to use during simulation)
                for i in range(self.numJoints):
                    p.resetJointState(self.kukaUid, i, jointPoses[i])
            # fingers
            p.setJointMotorControl2(self.kukaUid,
                                    7,
                                    p.POSITION_CONTROL,
                                    targetPosition=self.endEffectorAngle,
                                    force=self.maxForce)

        else:
            for action in range(len(motorCommands)):
                motor = self.motorIndices[action]
                p.setJointMotorControl2(self.kukaUid,
                                        motor,
                                        p.POSITION_CONTROL,
                                        targetPosition=motorCommands[action],
                                        force=self.maxForce)
