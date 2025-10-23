import pinocchio 
import math
import numpy as np
import sys
import os
import time
import math
from os.path import dirname, join, abspath
from pinocchio.visualize import RVizVisualizer
import math
 

""" # Load the urdf model
model = pinocchio.buildModelFromUrdf('franka_description/robots/frankaEmikaPanda.urdf')
print('model name: ' + model.name)

 
# Create data required by the algorithms
data = model.createData()
NQ = model.nq # dimension of the configuration vector of the robot
NV = model.nv # dimension of the velocity and acceleration vectors, and corresponds to the number of instantaneous DoF of the robot
print('Dimension of the configuration vector representation: ' + str(NQ))
print('Dimension of the velocity: ' + str(NV)) """


# Computes the forward kinematics at a random initial configuration and displays the position of each robot joint with its name
""" # Sample a random configuration
q = pinocchio.randomConfiguration(model)
print('q: %s' % q.T)
 
# Perform the forward kinematics over the kinematic tree
pinocchio.forwardKinematics(model,data,q)
 
# Print out the placement of each joint of the kinematic tree
for name, oMi in zip(model.names, data.oMi):
    print(("{:<24} : {: .2f} {: .2f} {: .2f}"
          .format( name, *oMi.translation.T.flat ))) """
    

# Runs the Recursive Newton-Euler Algorithm (RNEA) to compute the robot inverse dynamics.
# Inverse dynamics computes the needed torque to track a trajectory defined by the joint configuration, velocity and acceleration. 
# Velocity and acceleration are plain vectors, and can be initialized to any value. 
# Their dimension is model.nv, corresponding to the number of instantaneous degrees of freedom of the robot.
""" q = pinocchio.neutral(model)
print(q)
v = pinocchio.utils.zero(model.nv)
a = pinocchio.utils.zero(model.nv)
 
tau = pinocchio.rnea(model,data,q,v,a)
print('tau = ', tau.T) """


# ROBOT'S DYNAMICS MATRICES

""" # Compute robot's total mass
total_Mass = pinocchio.computeTotalMass(model,data) # Compute the total mass of the model, put it in data.mass[0] and return it.
# WARNING: This method does not fill the whole data.mass vector. Only data.mass[0] is updated. 
# If you need the whole data.mass vector to be computed, use computeSubtreeMasses 
# alternatively is possibile to use the function computeTotalMass(model) that computes the mass of the model and returns it.
print('Total mass of the model: ', data.mass[0])

# Generating joint position, angular velocity and angular acceleration using quintic polynomials
# 1. Generating position q, initial: 0 deg, end: 60 deg, ouput:rad
def mypoly_p(t):
  p = np.poly1d([12*60*math.pi/180/(2*3**5),-30*60*math.pi/180/(2*3**4),20*60*math.pi/180/(2*3**3),0,0,0])
  return p(t)

q = np.zeros((61, 9))
for i in range(0, 61):
    for j in range(7):
        q[i, j] = mypoly_p(0 + 3 / 60 * i) # 61*7 matrix, each column represents a sequence of joint

# 2. Generating angular velocity qdot, initial: 0 rad/s, end: rad/s, ouput:rad/s
def mypoly_v(t):
  p = np.poly1d([5*12*60*math.pi/180/(2*3**5),-4*30*60*math.pi/180/(2*3**4),3*20*60*math.pi/180/(2*3**3),0,0])
  return p(t)

qdot = np.zeros((61, 9))
for i in range(0, 61):
    for j in range(7):
        qdot[i, j] = mypoly_v(0 + 3 / 60 * i) # 61*7 matrix, each column represents a sequence of joint

# 3. Generating angular acceleration qddot, initial: 0 rad/s^2, end: rad/s^2, ouput:rad/s^2
def mypoly_a(t):
  p = np.poly1d([4*5*12*60*math.pi/180/(2*3**5),-3*4*30*60*math.pi/180/(2*3**4),2*3*20*60*math.pi/180/(2*3**3),0])
  return p(t)

qddot = np.zeros((61, 9))
for i in range(0, 61):
    for j in range(7):
        qddot[i, j] = mypoly_a(0 + 3 / 60 * i) # 61*7 matrix, each column represents a sequence of joint

# Calculates the torque of each joint, return 1*7 vector
# The necessary torque to follow the trajectory is computed using the RNEA algorithm ad we have seen above
Torque = np.zeros((61, 9))
for i in range(0,61):
    tau = pinocchio.rnea(model,data,q[i],qdot[i],qddot[i])   # 1*7 vector
    Torque[i][:] = tau.T
    #print('The ' + str(i) + 'th Torque is: ',i,tau.T)

# Computes the generalized gravity contribution G(q), stored in data.g
G_Torque = np.zeros((61, 9))
for i in range(0,61):
    G_Tau = pinocchio.computeGeneralizedGravity(model,data,q[i])
    G_Torque[i][:] = G_Tau
    #print('The ' + str(i) + 'th G_Tau is: ', G_Tau)


# Computes the upper triangular part of the joint space inertia matrix M, stored in data.M
M_Matrix = np.zeros((61,9,9))
for i in range(0,61):
    M_Temp = pinocchio.crba(model,data,q[i])
    M_Matrix[i,:,:] = M_Temp
    #print('The ' + str(i) + 'th M_Matrix is: ', M_Temp)

#Computes the Coriolis Matrix C
C_Matrix = np.zeros((61,9,9))
for i in range(0,61):
    C_Temp = pinocchio.computeCoriolisMatrix(model,data,q[i], qdot[i])
    C_Matrix[i,:,:] = C_Temp
    #print('The ' + str(i) + 'th C_Matrix is: ', C_Temp) """


# OPEN ROBOT WITH RVIZ:
abs_path = str(dirname(abspath(__file__)))

# Absolute path to urdf file:
urdf_filename = join(abs_path,"franka_description/robots/frankaEmikaPanda.urdf")

# Absolute path to franka_desription package (with meshes folder):
mesh_dir = abs_path

# Create robot model and connect collion and visual models
model, collision_model, visual_model = pinocchio.buildModelsFromUrdf(urdf_filename,mesh_dir) 
# Create data required by the algorithms
data, collision_data, visual_data = pinocchio.createDatas(model, collision_model, visual_model)


# Model dimension
q_dim = model.nq
print('Configuration vector size: ', q_dim)


# Pass the robot model to the RViz Viewer
viz = RVizVisualizer(model, collision_model, visual_model)

# Initialize the viewer
try:
    viz.initViewer()
except ImportError as err:
    print(err)
    sys.exit(0)

try:
    viz.loadViewerModel("pinocchio")
except AttributeError as err:
    print(err)
    sys.exit(0)

""" # Display neutral robot configuration
q0 = pinocchio.neutral(model)
print(q0)
viz.display(q0)

viz.sleep(5) # wait 5 seconds

# Display random robot configuration
q = pinocchio.randomConfiguration(model)
print(q)
viz.display(q) """

# Desired robot configuration
pi = math.pi

# Configuration with manipulator bent at 90 degrees grasping an object at mid-height:
#qdes = np.array([0, 0, 0, -pi/2, 0, pi/2, 1])
#q = np.concatenate((qdes,np.zeros(2)))

# Configuration with manipulator grasping object below its mid-height:
#qdes_init = np.array([0,0.7,0,-pi/2,0,1.5*pi/2,1])
#q = np.concatenate((qdes_init,np.zeros(2)))

# Configuration with manipulator end effector above its mid-height:
qdes_final = np.array([-pi/3,0,0,-pi/2,0,pi/2,1])
q = np.concatenate((qdes_final,np.zeros(2)))

# Display model
print(q)
viz.display(q)


# Perform the forward kinematics over the kinematic tree
frameId = model.getFrameId("panda_joint8")
print("ID FIXED JOINT 8:")
print(frameId)
pinocchio.forwardKinematics(model,data,q)
pinocchio.updateFramePlacements(model,data)

# Print out the placement of the end effector (we suppose now that the end effector is the last joint (8) of the arm)
# join7_name = model.names[7]
# print("NAME OF THE JOINT 7: ")
# print(join7_name)
# joint7_pos = data.oMi[7].translation    # position of joint7 wrt the world
# print("POSITION OF JOINT 7: ")
# print(joint7_pos)
joint8_pos = data.oMf[frameId].translation  # position of joint8: fixed joint at the end of the arm
print("POSITION OF FIXED JOINT 8:")
print(joint8_pos)


# Exit from rviz
input("Press enter to exit...")
viz.clean()



