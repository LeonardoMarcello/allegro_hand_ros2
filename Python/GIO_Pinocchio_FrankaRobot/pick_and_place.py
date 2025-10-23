import pinocchio 
import math
import numpy as np
import sys
import os
import time
import math
from os.path import dirname, join, abspath
from pinocchio.visualize import RVizVisualizer


def load_model(urdf_filename, mesh_dir):
    # Create robot model and connect collion and visual models
    model, collision_model, visual_model = pinocchio.buildModelsFromUrdf(urdf_filename,mesh_dir) 
    # Create data required by the algorithms
    data, collision_data, visual_data = pinocchio.createDatas(model, collision_model, visual_model)

    return model, collision_model, visual_model, data, collision_data, visual_data

def load_visualizer(model, collision_model, visual_model):
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



####################
# LOAD PANDA MODEL:
####################  
           
abs_path = str(dirname(abspath(__file__)))

# Absolute path to urdf file:
urdf_filename = join(abs_path,"franka_description/robots/frankaEmikaPanda.urdf")

# Absolute path to franka_description package (with meshes folder):
mesh_dir = abs_path

panda, collision_model, visual_model, data, collision_data, visual_data = load_model(urdf_filename,mesh_dir) 


##############################
# TRAJECTORY: PICK AND PLACE:
##############################
pi = math.pi
q_dim = panda.nq # dimension of the configuration vector of the robot

# Initial conditions
#q0 = pinocchio.neutral(panda)
q0 = np.concatenate((np.array([0,0.7,0,-pi/2,0,1.5*pi/2,1]),np.zeros(2)))
dq0 = np.zeros(q_dim)
ddq0 = np.zeros(q_dim)

# Desired final positions, velocities, accelerations
q_des = np.concatenate((np.array([-pi/3,0,0,-pi/2,0,pi/2,1]),np.zeros(2)))
#print(q_des)
dq_des = np.zeros(q_dim)
ddq_des = np.zeros(q_dim)

fs = 100        # frequency (Hz)
ts = 1/fs       # sample time (s)
t_start = 0     # initial simulation time (s)
t_end = 10      # final simulation time (s)
time_instants = np.arange(t_start, t_end+ts, ts)   # it goes from t_start to t_end

# The matrices for the reference are organized as follows: 
# number of rows = instants of time
# number of columns = dimension of the configuration vector
# so each row represents the robot configuration at the time associated to the row
q_ref = np.zeros((time_instants.size, q_dim))   # position reference values
dq_ref = np.zeros((time_instants.size, q_dim))    # velocity reference values
ddq_ref = np.zeros((time_instants.size, q_dim))   # acceleration reference values

index = 0
q_ref[index,:] = q0
dq_ref[index,:] = dq0
ddq_ref[index,:] = ddq0


# Get the POSITION of the END EFFECTOR at the INITIAL TIME wrt the world: end effector is the fixed joint panda_joint8
# In pinocchio library fixed joints are represented as frames
# Perform the forward kinematics over the kinematic tree
frameId = panda.getFrameId("panda_joint8")  # get fixed joint panda_joint8 frame ID
#print("ID FIXED JOINT 8:")
#print(frameId)
pinocchio.forwardKinematics(panda,data,q0)
pinocchio.updateFramePlacements(panda,data)

# Print out the placement of the end effector
joint8_pos_init = data.oMf[frameId].translation  # position of fixed joint panda_joint8
print("INITIAL POSITION OF FIXED JOINT 8:")
print(joint8_pos_init)


# Get the POSITION of the END EFFECTOR at the FINAL TIME wrt the world: end effector is the fixed joint panda_joint8
# In pinocchio library fixed joints are represented as frames
# Perform the forward kinematics over the kinematic tree
pinocchio.forwardKinematics(panda,data,q_des)
pinocchio.updateFramePlacements(panda,data)

# Print out the placement of the end effector
joint8_pos_final = data.oMf[frameId].translation  # position of fixed joint panda_joint8
print("FINAL POSITION OF FIXED JOINT 8:")
print(joint8_pos_final)


# Minimum jerk trajectory
# A task-space trajectory contains waypoints that represent end-effector motion in 3-D space. 
# Generate a minimum-jerk trajectory to connect waypoints in free space. 
# The purpose of this trajectory profile is to create a smooth trajectory with minimal jerky motion.
# Then generate the joint configurations of the Franka Emika Panda robot using inverse kinematics.






""" # SHOW MODEL IN RVIZ:
# Load RViz visualizer
load_visualizer(panda, collision_model, visual_model)

# qui mettere le istruzioni per mostrare il robot in rviz

# Exit from rviz
input("Press enter to exit...")
viz.clean() """

