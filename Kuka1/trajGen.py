import pybullet as p
import time
import math
from datetime import datetime
import pybullet_data

# clid = p.connect(p.SHARED_MEMORY)
# if (clid < 0):
#p.connect(p.GUI)
  #p.connect(p.SHARED_MEMORY_GUI)

# connect 
physicsClient =  p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) 
planeId = p.loadURDF("plane.urdf") 
kukaId = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0])

# rese base position and index to be 0,
p.resetBasePositionAndOrientation(kukaId, [0, 0, 0], [0, 0, 0, 1])

# double check that the the kuka was actually imported 
kukaEndEffectorIndex = 6
numJoints = p.getNumJoints(kukaId)
if (numJoints != 7):
  exit()

# will eventually want to be able to move the joints -- kuka_iiwa/model_free_base.urdf

# set limits for the null space (where the robot cannot go --> how is this determined 
#lower limits for null space
ll = [-.967, -2, -2.96, 0.19, -2.96, -2.09, -3.05]
#upper limits for null space
ul = [.967, 2, 2.96, 2.29, 2.96, 2.09, 3.05]
#joint ranges for null space
jr = [5.8, 4, 5.8, 4, 5.8, 4, 6]
#restposes for null space
rp = [0, 0, 0, 0.5 * math.pi, 0, -math.pi * 0.5 * 0.66, 0]
#joint damping coefficents
jd = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

# reset the joint states of the robot for each joint
for i in range(numJoints):
  p.resetJointState(kukaId, i, rp[i])

# no gravity simulation for now 
p.setGravity(0, 0, 0)

# starting time is 0
t = 0.

# set up previous positions, and the prior previous position
prevPose = [0, 0, 0]
prevPose1 = [0, 0, 0]
# does not have a prev pose, set this to false
hasPrevPose = 1

# use the null space 
useNullSpace = 1
# orientation points down if 1, not defined if 0
useOrientation = 1

#If we set useSimulation=0, it sets the arm pose to be the IK result directly without using dynamic control.
#This can be used to test the IK result accuracy.

# currently we are using dynamic control
useSimulation = 1 
# not using th computer's clock, so need to call the stepStimulation() command directly
useRealTimeSimulation = 0
p.setRealTimeSimulation(useRealTimeSimulation)
#trailDuration is duration (in seconds) after debug lines will be removed automatically
#use 0 for no-removal
trailDuration = 15

# ikSolver, not sure this will do
ikSolver = 0


i=0
while 1:
  i+=1
  # p.getCameraImage(320,
  #                 200,
  #                 flags=p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX,
  #                 renderer=p.ER_BULLET_HARDWARE_OPENGL)
  if (useRealTimeSimulation):
    # if computer's clock, get the  current time and add adjust 
    dt = datetime.now()
    t = (dt.second / 60.) * 2. * math.pi
  else:
    # otherwise just step the time parameter in increments of 0.01
    t = t + 0.01

  if (useSimulation and useRealTimeSimulation == 0):
    # not using dynamic control (?) and not using the computer's clock
    p.stepSimulation()

    
  for i in range(1): 
    # only 1 position at the moment, and the y, z are parametized by t
    pos = [-0.4, 0.2 * math.cos(t), 0. + 0.2 * math.sin(t)]
    # the orientation we want here is constant for all t, using euler angles to get the quaternion
    #end effector points down, not up (in case useOrientation==1)
    orn = p.getQuaternionFromEuler([0, -math.pi, 0])

    
    if (useNullSpace == 1):
      if (useOrientation == 1):
        # null space def AND if the end effector is pointing down, then calculate the iKin w/ the null space definition and desired ee orientation
        jointPoses = p.calculateInverseKinematics(kukaId, kukaEndEffectorIndex, pos, orn, ll, ul,
                                                  jr, rp)
      else:
        # null space def AND desired ee orientation is NOT down, then don't use the pre-defined orn
        jointPoses = p.calculateInverseKinematics(kukaId,
                                                  kukaEndEffectorIndex,
                                                  pos,
                                                  lowerLimits=ll,
                                                  upperLimits=ul,
                                                  jointRanges=jr,
                                                  restPoses=rp)
    else:
      if (useOrientation == 1):
        # null space not def AND ee down, then perform 100 iterations to determine the iKin with a threshold of 0.01, set the desired down orientation, and take into account damping forces 
        jointPoses = p.calculateInverseKinematics(kukaId,
                                                  kukaEndEffectorIndex,
                                                  pos,
                                                  orn,
                                                  jointDamping=jd,
                                                  solver=ikSolver,
                                                  maxNumIterations=100,
                                                  residualThreshold=.01)
      else:
        # null space not def AND we dont care about the orientation, then use an iKin solver
        jointPoses = p.calculateInverseKinematics(kukaId,
                                                  kukaEndEffectorIndex,
                                                  pos,
                                                  solver=ikSolver)

    if (useSimulation):
      for i in range(numJoints):
        # for each joint, set the joint motor controls using the solution to the inverse kinematics, using a simple P-controller, and a bit of force 
        p.setJointMotorControl2(bodyIndex=kukaId,
                                jointIndex=i,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=jointPoses[i],
                                targetVelocity=0,
                                force=500,
                                positionGain=0.03,
                                velocityGain=1)
    else:
      #reset the joint state (ignoring all dynamics, not recommended to use during simulation)
      for i in range(numJoints):
        p.resetJointState(kukaId, i, jointPoses[i])

  # get the current link state of ee
  ls = p.getLinkState(kukaId, kukaEndEffectorIndex)

  if (hasPrevPose):
    # if there are more positions (hasPrevPose ==1), then add info about it to the GUI
    p.addUserDebugLine(prevPose, pos, [0, 0, 0.3], 1, trailDuration)
    p.addUserDebugLine(prevPose1, ls[4], [1, 0, 0], 1, trailDuration)
 
  # set the prev pos to the current pose
  prevPose = pos

  # prevPose1 is the local orientation (expressed in the frame of the robot)
  prevPose1 = ls[4]
  # set has prev pose to 1
  hasPrevPose = 1

  
p.disconnect()