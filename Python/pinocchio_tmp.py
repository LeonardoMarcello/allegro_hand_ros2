import sys
from pathlib import Path
import numpy as np
from numpy.linalg import pinv
from scipy.linalg import block_diag
import time

import pinocchio
from pinocchio.utils import *
from pinocchio.visualize import GepettoVisualizer


# Load the robot model
# ---------------------------------------------------------------------------

mesh_dir = './description/urdf/meshes/'
urdf_model_path = './description/urdf/allegro_hand_description_right_B.urdf'
# Build robot wrapper
ahand = pinocchio.robot_wrapper.RobotWrapper.BuildFromURDF(urdf_model_path, mesh_dir)#, pinocchio.JointModelFreeFlyer())
print("model name: " + ahand.model.name)
# Check dimensions of the original model
print(f"Number of joints: {ahand.model.njoints}")
for jn in ahand.model.joints:
    print(jn)
print("-" * 30)

print(f"Number of dofs: {ahand.model.nq}")              # dimension of the configuration vector of the robot
print(f"Number of frames: {ahand.model.nframes}")
print(f"Number of actuated dofs: {ahand.model.nv}")     # dimension of the velocity and acceleration vectors, and corresponds to the number of instantaneous DoF of the robot

# Print out the placement of each joint/frame of the kinematic tree
q = pinocchio.randomConfiguration(ahand.model)

# Compute placements
# > joints
pinocchio.forwardKinematics(ahand.model,ahand.data,q)
# ahand.data.oMi[idx]           -> placement of Joint idx in the world frame {O} (call forwardKinematics before)
# ahand.com(q)                  -> Compute the robot center of mass.
# ahand.placement(q,joint_idx)  -> Compute the placement of joint_idx

# > Frames
pinocchio.updateFramePlacements(ahand.model, ahand.data)
# ahand.data.oMf[idx] -> placement of Frame idx in the world frame {O} (call updateFramePlacements before)

# print placements
print("Joint placements:")
for name, oMi in zip(ahand.model.names, ahand.data.oMi):
    print(("{:<24} : {: .2f} {: .2f} {: .2f}"
          .format( name, *oMi.translation.T.flat)))

print("Frames placements:")
for i, oMf in zip(range(len(ahand.model.frames)), ahand.data.oMf):
    print(("{:<24} : {: .2f} {: .2f} {: .2f}, parent: '{}', type: {}"
          .format( ahand.model.frames[i].name, *oMf.translation.T.flat, ahand.model.names[ahand.model.frames[i].parent], ahand.model.frames[i].type )))
print("-" * 30)


# Frames indexing
# ---------------------------------------------------------------------------
# (i) indice: 3_0_tip
# (ii) medio: 7_0_tip
# (iii) anulare: 11_0_tip
# (iv) pollice: 15_0_tip

# Joint indeces
idx_index = ahand.index('joint_3_0') # red
idx_middle = ahand.index('joint_7_0') # green
idx_ring = ahand.index('joint_11_0') # blue
idx_thumb = ahand.index('joint_15_0') # cyan
# Frame indeces
idx_index = ahand.model.getFrameId('joint_3_0_tip') # red
idx_middle = ahand.model.getFrameId('joint_7_0_tip') # green
idx_ring = ahand.model.getFrameId('joint_11_0_tip') # blue
idx_thumb = ahand.model.getFrameId('joint_15_0_tip') # cyan

print(f"Index finger joint index: {idx_index}")
print(f"Middle finger joint index: {idx_middle}")
print(f"Ring finger joint index: {idx_ring}")
print(f"Thumb finger joint index: {idx_thumb}")


# Visualization via gepetto-viewer
# ---------------------------------------------------------------------------
# Visualize the model in gepetto-viewer (run `gepetto-gui` on another terminal)
ahand.initViewer(loadModel=True)
ahand.loadViewerModel("pinocchio")
ahand.display(q)

## Get Robot element
#visualObj = ahand.visual_model.geometryObjects[4]  # 3D object representing the robot forarm
#visualName = visualObj.name                        # Name associated to this object
#visualRef = ahand.getViewerNodeName(visualObj, pinocchio.GeometryType.VISUAL)    # Viewer reference (string) representing this object
## Move that element
#q1 = (1, 1, 1, 1, 0, 0, 0)  # x, y, z, quaternion
#ahand.viewer.gui.applyConfiguration(visualRef, q1)
#ahand.viewer.gui.refresh()  # Refresh the window.

# Add sphere
rgbt = [1.0, 0.2, 0.2, 1.0]  # red, green, blue, transparency
ahand.viewer.gui.addSphere("world/sphere", .01, rgbt)  # .1 is the radius
rgbt1 = [1, 0.2, 0.2, 1.0]  # red, green, blue, transparency
rgbt2 = [0.2, 1, 0.2, 1.0]  # red, green, blue, transparency
rgbt3 = [0.2, 0.2, 1, 1.0]  # red, green, blue, transparency
rgbt4 = [0.2, 1, 1, 1.0]  # red, green, blue, transparency
ahand.viewer.gui.addSphere("world/sphere1", .014, rgbt1)  # .015 is the radius
ahand.viewer.gui.addSphere("world/sphere2", .014, rgbt2)  # .015 is the radius
ahand.viewer.gui.addSphere("world/sphere3", .014, rgbt3)  # .015 is the radius
ahand.viewer.gui.addSphere("world/sphere4", .014, rgbt4)  # .015 is the radius

rgbt1_t = [1, 0.2, 0.2, 0.5]  # red, green, blue, transparency
rgbt2_t = [0.2, 1, 0.2, 0.5]  # red, green, blue, transparency
rgbt3_t = [0.2, 0.2, 1, 0.5]  # red, green, blue, transparency
rgbt4_t = [0.2, 1, 1, 0.5]  # red, green, blue, transparency
ahand.viewer.gui.addSphere("world/sphere1_target", .014, rgbt1_t)  # .015 is the radius
ahand.viewer.gui.addSphere("world/sphere2_target", .014, rgbt2_t)  # .015 is the radius
ahand.viewer.gui.addSphere("world/sphere3_target", .014, rgbt3_t)  # .015 is the radius
ahand.viewer.gui.addSphere("world/sphere4_target", .014, rgbt3_t)  # .015 is the radius

ahand.viewer.gui.applyConfiguration("world/sphere", (.5, .1, .2, 1.,0.,0.,0. ))
ahand.viewer.gui.refresh()  # Refresh the window.

# ---------------------------------------------------------------------------
# KINEMATICS
# ---------------------------------------------------------------------------
# compute Jacobian csi = J(q)dq
# csi_i = [v_i',w_i']'
# pinocchio.computeJointJacobian(model, data, q, joint_id) -> Jacobian in local frame
# pinocchio.computeFrameJacobian(model, data, q, body_id, reference_frame) -> Jacobian in reference frame
# Refrerence Frames pinocchio.ReferenceFrame.*:
# - LOCAL: the Jacobian is expressed in the local frame of the joint/body
# - LOCAL_WORLD_ALIGNED: the Jacobian is expressed in a frame with the same
# - WORLD: the Jacobian is expressed in the world frame
def computeJacobian(q):
    #J_i = pinocchio.computeJointJacobian(ahand.model, ahand.data, q, idx_index) # Oss. it is in ee frame 
    #J_ii = pinocchio.computeJointJacobian(ahand.model, ahand.data, q, idx_middle)
    #J_iii = pinocchio.computeJointJacobian(ahand.model, ahand.data, q, idx_ring)
    #J_iv = pinocchio.computeJointJacobian(ahand.model, ahand.data, q, idx_thumb)
    J_i = pinocchio.computeFrameJacobian(ahand.model, ahand.data, q, idx_index, pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    J_ii = pinocchio.computeFrameJacobian(ahand.model, ahand.data, q, idx_middle, pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    J_iii = pinocchio.computeFrameJacobian(ahand.model, ahand.data, q, idx_ring, pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    J_iv = pinocchio.computeFrameJacobian(ahand.model, ahand.data, q, idx_thumb, pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    J = np.vstack([J_i, J_ii, J_iii, J_iv])
    return J

def computeForwardKinematic(q):
    pinocchio.forwardKinematics(ahand.model, ahand.data, q)     # Compute joint placements
    #x_i = np.vstack([ahand.data.oMf[idx_index].translation.copy(), ahand.data.oMf[idx_index].rotation.copy()])
    #x_ii = np.vstack([ahand.data.oMf[idx_middle].translation.copy(), ahand.data.oMf[idx_middle].rotation.copy()])
    #x_iii = np.vstack([ahand.data.oMf[idx_ring].translation.copy(), ahand.data.oMf[idx_ring].rotation.copy()])
    #x_iv = np.vstack([ahand.data.oMf[idx_thumb].translation.copy(), ahand.data.oMf[idx_thumb].rotation.copy()])
    x_i = ahand.data.oMf[idx_index].translation.copy()
    x_ii = ahand.data.oMf[idx_middle].translation.copy()
    x_iii = ahand.data.oMf[idx_ring].translation.copy()
    x_iv = ahand.data.oMf[idx_thumb].translation.copy()
    x = np.hstack([x_i, x_ii, x_iii, x_iv])
    return x

q = zero(ahand.model.nq)
q[5] = np.pi/2
pinocchio.forwardKinematics(ahand.model, ahand.data, q)     # Compute joint placements
pinocchio.updateFramePlacements(ahand.model, ahand.data)     # Compute frame placements
ahand.display(q)

J = computeJacobian(q)
print(f"J shape: {J.shape}")
print(f"q: {q}")
print(f"Ji(0) shape: {J[0:6,0:4]}")
print(f"Jii(0) shape: {J[6:12,8:12]}")
print(f"Jiii(0) shape: {J[12:18,12:16]}")
print(f"Jiv(0) shape: {J[18:25,4:8]}")
time.sleep(5)

# Trajectory in the operational space
# initial position of fingertips
q = rand(ahand.model.nq)
pinocchio.forwardKinematics(ahand.model, ahand.data, q)     # Compute joint placements
pinocchio.updateFramePlacements(ahand.model, ahand.data)     # Compute frame placements
ahand.display(q)

#x_i_0 = ahand.data.oMi[idx_index].translation.copy()
#x_ii_0 = ahand.data.oMi[idx_middle].translation.copy()
#x_iii_0 = ahand.data.oMi[idx_ring].translation.copy()
#x_iv_0 = ahand.data.oMi[idx_thumb].translation.copy()
x_i_0 = ahand.data.oMf[idx_index].translation.copy()
x_ii_0 = ahand.data.oMf[idx_middle].translation.copy()
x_iii_0 = ahand.data.oMf[idx_ring].translation.copy()
x_iv_0 = ahand.data.oMf[idx_thumb].translation.copy()
x_center = (x_i_0 + x_ii_0 + x_iii_0 + x_iv_0)/4

#x_0 = np.hstack([x_i_0,1,0,0,0,       x_ii_0,1,0,0,0,   x_iii_0,1,0,0,0,  x_iv_0,1,0,0,0])
#x_f = np.hstack([x_center,1,0,0,0,    x_center,1,0,0,0, x_center,1,0,0,0, x_center,1,0,0,0])
x_0 = np.hstack([x_i_0,x_ii_0,x_iii_0,x_iv_0])
x_f = np.hstack([x_center,x_center,x_center,x_center])
print(f"x_0: {x_0.shape}")

ahand.viewer.gui.applyConfiguration("world/sphere", (x_center[0], x_center[1], x_center[2], 1.0 ,0.,0.,0. ))
ahand.viewer.gui.applyConfiguration("world/sphere1", (x_i_0[0], x_i_0[1], x_i_0[2], 1.0 ,0.,0.,0. ))
ahand.viewer.gui.applyConfiguration("world/sphere2", (x_ii_0[0], x_ii_0[1], x_ii_0[2], 1.0 ,0.,0.,0. ))
ahand.viewer.gui.applyConfiguration("world/sphere3", (x_iii_0[0], x_iii_0[1], x_iii_0[2], 1.0 ,0.,0.,0. ))
ahand.viewer.gui.applyConfiguration("world/sphere4", (x_iv_0[0], x_iv_0[1], x_iv_0[2], 1.0 ,0.,0.,0. ))
ahand.viewer.gui.applyConfiguration("world/sphere1_target", (x_i_0[0], x_i_0[1], x_i_0[2], 1.0 ,0.,0.,0. ))
ahand.viewer.gui.applyConfiguration("world/sphere2_target", (x_ii_0[0], x_ii_0[1], x_ii_0[2], 1.0 ,0.,0.,0. ))
ahand.viewer.gui.applyConfiguration("world/sphere3_target", (x_iii_0[0], x_iii_0[1], x_iii_0[2], 1.0 ,0.,0.,0. ))
ahand.viewer.gui.applyConfiguration("world/sphere4_target", (x_iv_0[0], x_iv_0[1], x_iv_0[2], 1.0 ,0.,0.,0. ))
ahand.viewer.gui.refresh()  # Refresh the window.

time.sleep(10)
dt = 0.01
for i in range(10000):
    omega = 0.1
    theta = i*dt*omega

    x_t = x_0 + (1 - np.cos(theta))*(x_f - x_0)/2
    dx_t = (omega*np.sin(theta))*(x_f-x_0)/2   # desired trajectory

    J = computeJacobian(q)
    J = J[[0,1,2,   6,7,8,    12,13,14, 18,19,20]]  # jacobian of linear velocity jacobian of i,ii,iii,iv fingertips


    # Compute SVD to check conditioning
    U, s, Vt = np.linalg.svd(J, full_matrices=False)
    cond_number = s.max() / s.min() if s.min() > 0 else np.inf
    #if cond_number > 1e-12:
    #    print(f"Matrix is ill-conditioned (cond={cond_number:.2e})")
    if s.min() < 1e-6:
        print(f"Matrix is singular (smallest singular value={s.min():.2e})")

    # Control law
    csi = computeForwardKinematic(q)
    e = x_t - csi
    dq = pinv(J).dot(dx_t + e)# damped pseudo-inverse


    q = pinocchio.integrate(ahand.model, q, dq*dt)
    pinocchio.forwardKinematics(ahand.model, ahand.data, q)     # Compute joint placements
    pinocchio.updateFramePlacements(ahand.model, ahand.data)    # Also compute operational frame placements


    #ahand.viewer.gui.applyConfiguration("world/sphere1", (x_t[0], x_t[1], x_t[2], 1.0 ,0.,0.,0. ))
    #ahand.viewer.gui.applyConfiguration("world/sphere2", (x_t[6], x_t[7], x_t[8], 1.0 ,0.,0.,0. ))
    #ahand.viewer.gui.applyConfiguration("world/sphere3", (x_t[12], x_t[13], x_t[14], 1.0 ,0.,0.,0. ))
    #ahand.viewer.gui.applyConfiguration("world/sphere4", (x_t[18], x_t[19], x_t[20], 1.0 ,0.,0.,0. ))
    ahand.viewer.gui.applyConfiguration("world/sphere1", (csi[0], csi[1], csi[2], 1.0 ,0.,0.,0. ))
    ahand.viewer.gui.applyConfiguration("world/sphere2", (csi[3], csi[4], csi[5], 1.0 ,0.,0.,0. ))
    ahand.viewer.gui.applyConfiguration("world/sphere3", (csi[6], csi[7], csi[8], 1.0 ,0.,0.,0. ))
    ahand.viewer.gui.applyConfiguration("world/sphere4", (csi[9], csi[10],csi[11], 1.0 ,0.,0.,0. ))
    ahand.viewer.gui.applyConfiguration("world/sphere1_target", (x_t[0], x_t[1], x_t[2], 1.0 ,0.,0.,0. ))
    ahand.viewer.gui.applyConfiguration("world/sphere2_target", (x_t[3], x_t[4], x_t[5], 1.0 ,0.,0.,0. ))
    ahand.viewer.gui.applyConfiguration("world/sphere3_target", (x_t[6], x_t[7], x_t[8], 1.0 ,0.,0.,0. ))
    ahand.viewer.gui.applyConfiguration("world/sphere4_target", (x_t[9], x_t[10], x_t[11], 1.0 ,0.,0.,0. ))
    ahand.viewer.gui.refresh()  # Refresh the window.
    ahand.display(q)

    time.sleep(dt)
    #time.sleep(5)
    #print('-'*20)


# ---------------------------------------------------------------------------
# DYNAMICS
# ---------------------------------------------------------------------------
# M ddq + b = tau = M(q)ddq + C(q,dq)dq + G = tau
# compute dynamic drift -- Coriolis, centrifugal, gravity
b = pinocchio.rnea(ahand.model, ahand.data, q, vq, aq0)
print(f"b shape: {b.shape}")
#print(f"b: {b}")
# compute mass matrix M
M = pinocchio.crba(ahand.model, ahand.data, q)
print(f"M shape: {M.shape}")
#print(f"M: {M}")