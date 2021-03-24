import os
from time import sleep
import numpy as np
import pybullet as p
import pybullet_data

obstacle_number = (3, 10)
obstacle_range = ()

visualShift = [0, 0, 0]
shift = [0, 0, 0]
meshScale = [0.1, 0.1, 0.1]

p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
urId = p.loadURDF("UR5/ur_e_description/urdf/ur5e.urdf", [0, 0, 0], useFixedBase=True)
urEndEffectorIndex = 6
pose_range = [(p.getJointInfo(urId, jointId)[8], p.getJointInfo(urId, jointId)[9]) for jointId in range(p.getNumJoints(urId))]

#p.loadURDF("Ball.urdf", [2, 2, 5])


def create_voxel(halfExtents, basePosition):

    groundColId = p.createCollisionShape(p.GEOM_BOX, halfExtents=halfExtents)
    groundVisID = p.createVisualShape(shapeType=p.GEOM_BOX,
                                      rgbaColor=[1, 1, 1, 1],
                                      specularColor=[0.4, .4, 0],
                                      halfExtents=halfExtents)
    groundId = p.createMultiBody(baseMass=0,
                                 baseCollisionShapeIndex=groundColId,
                                 baseVisualShapeIndex=groundVisID,
                                 basePosition=basePosition)
    return groundId


create_voxel([0.1, 0.1, 0.1], [0, 0, 1])
p.setGravity(0, 0, -10)

joint_pose = np.random.uniform(size=p.getNumJoints(urId))
joint_pose = np.array(pose_range)[:, 0] + (np.array(pose_range)[:, 1] - np.array(pose_range)[:, 0]) * joint_pose

space_max = [-1, -1, -1]
space_min = [1, 1, 1]

for _ in range(100000):
    p.stepSimulation()

    joint_pose = np.random.uniform(size=p.getNumJoints(urId))
    joint_pose = np.array(pose_range)[:, 0] + (np.array(pose_range)[:, 1] - np.array(pose_range)[:, 0]) * joint_pose
    for i in range(p.getNumJoints(urId)):
        p.resetJointState(urId, i, joint_pose[i])
    link_state = p.getLinkState(urId, urEndEffectorIndex)[0]
    space_max = np.maximum(np.array(space_max), np.array(link_state))
    space_min = np.minimum(np.array(space_min), np.array(link_state))

print(space_max)  # [0.91640153 0.91834576 1.28094257]
print(space_min)  # [-0.91767151 -0.91807415 -0.34391806]
