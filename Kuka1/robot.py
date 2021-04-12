import pybullet
import numpy as np
from helper import midpoint, boundBox, drawCont
import math


class Robot:
    """
    Control the motion of the robot 
    """
    def __init__(self, p, urdfPath, initPos):
        """Initialize the robot model.
        Args:
            p: pybullet library object
            urdf_path: string of the path to the robot urdf file
            base_pos: the robot base position
            base_orientation: the robot base orientation as a quaternion
        """
        # use the pybullet library 
        self.p = p
        # load the robot 
        self.id = self.p.loadURDF(urdfPath)
        self.numJoints = self.p.getNumJoints(self.id)

        # info aboout the kuka iiwa specifically 
        self.endEffectorIndex = 6
        self.restOrn = [0, 0, 0, 1]
        # set limits for the null space 
        self.ll = [-.967, -2, -2.96, 0.19, -2.96, -2.09, -3.05] # lower limits
        self.ul = [.967, 2, 2.96, 2.29, 2.96, 2.09, 3.05] # upper limits
        self.jr = [5.8, 4, 5.8, 4, 5.8, 4, 6] # joint ranges 
        self.rp = [0, 0, 0, 0.5 * math.pi, 0, -math.pi * 0.5 * 0.66, 0] # rest poses

        # set to desired starting position, with end effector down 
        self.p.resetBasePositionAndOrientation(self.id, posObj=initPos, ornObj=self.restOrn)
        # start tracking the robots's position
        self.baseInfo = self.p.getLinkState(bodyUniqueId=self.id, linkIndex=0)
        # make a bounding box of the robot's workspace (CW, NE->NW)
        self.vertices = [0.5, -1.5, 0.5, -0.5, -0.5, -0.5, -0.5, -1.5]
        (self.rBB, self.rBBC) = boundBox(self.baseInfo[0], self.vertices)
        # # show the bounding box --> actually handle this in main...
        self.initBB = drawCont(self.p, self.rBB, [0,0,0])

        

    def translate(self, vel):
        # delete initial bounding box
        # self.p.removeUserDebugItem(self.initBB)
        # perform the motion 
        self.p.resetBaseVelocity(objectUniqueId = self.id, linearVelocity=vel)
        # # update robot's location 
        self.baseInfo = self.p.getLinkState(bodyUniqueId=self.id, linkIndex=0)
        # # update the bounding box
        (self.rBB, self.rBBC) = boundBox(self.baseInfo[0], self.vertices)
        # should this return?

    def execTraj(self, points):
        "'3D Print' the required joints"
        # want the end-eff pointing down 
        orn = self.p.getQuaternionFromEuler([0, -math.pi, 0])
        # calculate the invese kinematics of a dataframe of a group of points and turn into a list of joint configurations 
        configs = []
        for pos in range(len(points)):
            # print(points.iloc[pos])
            jointPoses = self.p.calculateInverseKinematics(bodyUniqueId=self.id, 
            endEffectorLinkIndex=self.endEffectorIndex, 
            targetPosition=points.iloc[pos],
            targetOrientation=orn, 
            lowerLimits=self.ll, 
            upperLimits=self.ul, 
            jointRanges=self.jr,  
            restPoses=self.rp)
            configs.append(jointPoses)
        # go through each configuation and execute the trajectories
        for n in (range(len(configs))):
            # get a configuration in the list of configurations 
            config = configs[n]
            # motor control for each joint 
            for joint in range(self.numJoints):
                    self.p.setJointMotorControl2(
                    bodyIndex = self.id, 
                    jointIndex=joint, 
                    controlMode = self.p.POSITION_CONTROL, targetPosition=config[joint])
        return





