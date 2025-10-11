import sys
from pathlib import Path
import numpy as np
from numpy.linalg import pinv
from scipy.linalg import block_diag
import time
from enum import Enum

import pinocchio
from pinocchio.utils import *
from pinocchio.visualize import GepettoVisualizer
from pinocchio.robot_wrapper import RobotWrapper 
from pinocchio.shortcuts import buildModelsFromUrdf, createDatas

MESH_DIR = './description/urdf/meshes/'
URDF_PATH = './description/urdf/allegro_hand_description_right_B.urdf'
XML_PATH = './description/urdf/allegro_hand_description_right_B.xml'

class AHAND(RobotWrapper):

    class Finger(Enum):
        INDEX = 1
        MIDDLE = 2
        RING = 3
        THUMB = 4

    class JacobianType(Enum):
        POSITION = 1
        LINEAR_VELOCITY = 1
        FULL = 2
        INDEX = 3
        MIDDLE = 4
        RING = 5
        THUMB = 6

    # Joint limits from URDF
    JOINT_LIMITS = np.array([
        [-0.3, 0.3],    # Index 0
        [-0.01, 1.6],   # Index 1
        [-0.07, 1.86],  # Index 2
        [-0.02, 2.01],  # Index 3
        [0.00, 1.78],   # Thumb 0
        [-0.26, 1.65],  # Thumb 1
        [-0.05, 1.85],  # Thumb 2
        [-0.09, 1.80],  # Thumb 3
        [-0.26, 0.26],  # Middle 0
        [-0.21, 1.79],  # Middle 1
        [-0.12, 1.86],  # Middle 2
        [-0.21, 1.85],  # Middle 3
        [-0.26, 0.29],  # Ring 0
        [-0.21, 1.79],  # Ring 1
        [-0.12, 1.86],  # Ring 2
        [-0.21, 1.85],  # Ring 3
    ])

    def __init__(self, urdf_path, mesh_dir):
        # Build robot wrapper
        #self.ahand = RobotWrapper.BuildFromURDF(urdf_path, mesh_dir)#, pinocchio.JointModelFreeFlyer())
        model, collision_model, visual_model = buildModelsFromUrdf(urdf_path, mesh_dir, None, False, None)
        RobotWrapper.__init__(self,model=model,collision_model=collision_model,visual_model=visual_model)

        self.end_effector_indeces = {
                'index' :   self.model.getFrameId('joint_3_0_tip'), # red
                'middle':   self.model.getFrameId('joint_7_0_tip'), # green
                'ring'  :   self.model.getFrameId('joint_11_0_tip'), # blue
                'thumb' :   self.model.getFrameId('joint_15_0_tip') # cyan
        }

        # State:           Index (i)            Thumb (iv)            Middle (ii)             Ring(iii)
        # q =       [q_0,q_1,q_2,q_3,      q_12,q_13,q_14,q_15     q_4,q_5,q_6,q_7,        q_8,q_9,q_10,q_11]
        self.q = zero(self.model.nq)
        self.dq = zero(self.model.nv)
        self.ddq = zero(self.model.nv)
        self.x_t = zero(12)

        return
    def __del__(self):
        #self.viewer.gui.deleteNode("world/pinocchio",True)
        #self.viewer.gui.deleteNode("world/sphere1",True)
        #self.viewer.gui.deleteNode("world/sphere2",True)
        #self.viewer.gui.deleteNode("world/sphere3",True)
        #self.viewer.gui.deleteNode("world/sphere4",True)
        #self.viewer.gui.deleteNode("world/sphere1_target", True)
        #self.viewer.gui.deleteNode("world/sphere2_target", True)
        #self.viewer.gui.deleteNode("world/sphere3_target", True)
        #self.viewer.gui.deleteNode("world/sphere4_target", True)
        #self.viewer.gui.refresh()
        return

    def set_index(self, qi, dqi=None, ddqi=None):
        self.q[0:4] = qi
        if dqi is not None:
            self.dq[0:4] = dqi
        if ddqi is not None:
            self.ddq[0:4] = ddqi
    def set_middle(self, qiii, dqiii=None, ddqiii=None):
        self.q[8:12] = qiii
        if dqiii is not None:
            self.dq[8:12] = dqiii
        if ddqiii is not None:
            self.ddq[8:12] = ddqiii
    def set_ring(self, qiv, dqiv=None, ddqiv=None):
        self.q[12:16] = qiv
        if dqiv is not None:
            self.dq[12:16] = dqiv
        if ddqiv is not None:
            self.ddq[12:16] = ddqiv
    def set_thumb(self, qii, dqii=None, ddqii=None):
        self.q[4:8] = qii
        if dqii is not None:
            self.dq[4:8] = dqii
        if ddqii is not None:
            self.ddq[4:8] = ddqii

    def set_target(self, q):
        real_q = self.q
        self.q = q
        self.updateConfig()
        self.x_t = self.computeForwardKinematic(type=self.JacobianType.POSITION).tolist()
        self.q = real_q
        self.updateConfig()

    """
    Viewer
    """
    def updateConfig(self):
        # Compute placements
        # > joints
        pinocchio.forwardKinematics(self.model,self.data, self.q)
        # ahand.data.oMi[idx]           -> placement of Joint idx in the world frame {O} (call forwardKinematics before)
        # ahand.com(q)                  -> Compute the robot center of mass.
        # ahand.placement(q,joint_idx)  -> Compute the placement of joint_idx
        # --------------------------------------------
        # > Frames
        pinocchio.updateFramePlacements(self.model, self.data)
        # ahand.data.oMf[idx] -> placement of Frame idx in the world frame {O} (call updateFramePlacements before)
        return

    def initGepettoViewer(self):
        # hand
        self.initViewer(loadModel=True)
        self.loadViewerModel("pinocchio")

        #end-effector spheres
        rgbt1 = [1.0, 0.2, 0.2, 1.0]  # red, green, blue, transparency
        rgbt2 = [0.2, 1.0, 0.2, 1.0]
        rgbt3 = [0.2, 0.2, 1.0, 1.0]
        rgbt4 = [0.2, 1.0, 1.0, 1.0]
        self.viewer.gui.addSphere("world/sphere1", .014, rgbt1)  # .015 is the radius
        self.viewer.gui.addSphere("world/sphere2", .014, rgbt2)
        self.viewer.gui.addSphere("world/sphere3", .014, rgbt3)
        self.viewer.gui.addSphere("world/sphere4", .014, rgbt4)

        # target spheres
        rgbt1_t = [1.0, 0.2, 0.2, 0.5]  # red, green, blue, transparency
        rgbt2_t = [0.2, 1.0, 0.2, 0.5]
        rgbt3_t = [0.2, 0.2, 1.0, 0.5]
        rgbt4_t = [0.2, 1.0, 1.0, 0.5]
        self.viewer.gui.addSphere("world/sphere1_target", .014, rgbt1_t)
        self.viewer.gui.addSphere("world/sphere2_target", .014, rgbt2_t)
        self.viewer.gui.addSphere("world/sphere3_target", .014, rgbt3_t)
        self.viewer.gui.addSphere("world/sphere4_target", .014, rgbt4_t)

        # shows
        self.viewer.gui.refresh()  # Refresh the window
        self.display(self.q)

    def show(self, target = False):
        if target:
            csi = self.computeForwardKinematic(type=self.JacobianType.POSITION).tolist()
            self.viewer.gui.applyConfiguration("world/sphere1", (csi[0], csi[1], csi[2], 1.0 ,0.,0.,0. ))
            self.viewer.gui.applyConfiguration("world/sphere2", (csi[3], csi[4], csi[5], 1.0 ,0.,0.,0. ))
            self.viewer.gui.applyConfiguration("world/sphere3", (csi[6], csi[7], csi[8], 1.0 ,0.,0.,0. ))
            self.viewer.gui.applyConfiguration("world/sphere4", (csi[9], csi[10],csi[11], 1.0 ,0.,0.,0. ))
            self.viewer.gui.applyConfiguration("world/sphere1_target", (self.x_t[0], self.x_t[1], self.x_t[2], 1.0 ,0.,0.,0. ))
            self.viewer.gui.applyConfiguration("world/sphere2_target", (self.x_t[3], self.x_t[4], self.x_t[5], 1.0 ,0.,0.,0. ))
            self.viewer.gui.applyConfiguration("world/sphere3_target", (self.x_t[6], self.x_t[7], self.x_t[8], 1.0 ,0.,0.,0. ))
            self.viewer.gui.applyConfiguration("world/sphere4_target", (self.x_t[9], self.x_t[10],self.x_t[11], 1.0 ,0.,0.,0. ))
        self.viewer.gui.refresh()
        self.display(self.q)

    """
    ---------------------------------------------------------------------------
    Kinematics
    ---------------------------------------------------------------------------
    compute forward kinematics csi = Q(q)
    csi_i = [p',q']'

    compute Jacobian dcsi = J(q)dq
    dcsi_i = [v_i',w_i']'
    pinocchio.computeJointJacobian(model, data, q, joint_id) -> Jacobian in local frame
    pinocchio.computeFrameJacobian(model, data, q, body_id, reference_frame) -> Jacobian in reference frame
    Refrerence Frames pinocchio.ReferenceFrame.*:
    - LOCAL: the Jacobian is expressed in the local frame of the joint/body
    - LOCAL_WORLD_ALIGNED: the Jacobian is expressed in a frame with the same
    - WORLD: the Jacobian is expressed in the world frame
    """

    def computeForwardKinematic(self, type=JacobianType.POSITION):
        if type == self.JacobianType.POSITION:
            x_i = self.data.oMf[self.end_effector_indeces['index']].translation.copy()
            x_ii = self.data.oMf[self.end_effector_indeces['middle']].translation.copy()
            x_iii = self.data.oMf[self.end_effector_indeces['ring']].translation.copy()
            x_iv = self.data.oMf[self.end_effector_indeces['thumb']].translation.copy()
            x = np.hstack([x_i, x_ii, x_iii, x_iv])
            return x
        elif type == self.JacobianType.INDEX:
            x_i = np.vstack([self.data.oMf[self.end_effector_indeces['index']].translation.copy(), self.data.oMf[self.end_effector_indeces['index']].rotation.copy()])
            return x_i
        elif type == self.JacobianType.MIDDLE:
            x_ii = np.vstack([self.data.oMf[self.end_effector_indeces['middle']].translation.copy(), self.data.oMf[self.end_effector_indeces['middle']].rotation.copy()])
            return x_ii
        elif type == self.JacobianType.RING:
            x_iii = np.vstack([self.data.oMf[self.end_effector_indeces['ring']].translation.copy(), self.data.oMf[self.end_effector_indeces['ring']].rotation.copy()])
            return x_iii
        elif type == self.JacobianType.THUMB:
            x_iv = np.vstack([self.data.oMf[self.end_effector_indeces['thumb']].translation.copy(), self.data.oMf[self.end_effector_indeces['thumb']].rotation.copy()])
            return x_iv

        x_i = np.vstack([self.data.oMf[self.end_effector_indeces['index']].translation.copy(), self.data.oMf[self.end_effector_indeces['index']].rotation.copy()])
        x_ii = np.vstack([self.data.oMf[self.end_effector_indeces['middle']].translation.copy(), self.data.oMf[self.end_effector_indeces['middle']].rotation.copy()])
        x_iii = np.vstack([self.data.oMf[self.end_effector_indeces['ring']].translation.copy(), self.data.oMf[self.end_effector_indeces['ring']].rotation.copy()])
        x_iv = np.vstack([self.data.oMf[self.end_effector_indeces['thumb']].translation.copy(), self.data.oMf[self.end_effector_indeces['thumb']].rotation.copy()])
        x = np.hstack([x_i, x_ii, x_iii, x_iv])
        return x

    def computeJacobian(self, type=JacobianType.FULL):
        if type == self.JacobianType.LINEAR_VELOCITY:
            J_i = pinocchio.computeFrameJacobian(self.model, self.data, self.q, self.end_effector_indeces['index'], pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED)[0:3,:]
            J_ii = pinocchio.computeFrameJacobian(self.model, self.data, self.q, self.end_effector_indeces['middle'], pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED)[0:3,:]
            J_iii = pinocchio.computeFrameJacobian(self.model, self.data, self.q, self.end_effector_indeces['ring'], pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED)[0:3,:]
            J_iv = pinocchio.computeFrameJacobian(self.model, self.data, self.q, self.end_effector_indeces['thumb'], pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED)[0:3,:]

            J = np.vstack([J_i, J_ii, J_iii, J_iv])
            J = J[[0,1,2,   6,7,8,    12,13,14, 18,19,20]]  # select linear velocity jacobian of i,ii,iii,iv fingertips
            return J
        elif type == self.JacobianType.INDEX:
            return pinocchio.computeFrameJacobian(self.model, self.data, self.q, self.end_effector_indeces['index'], pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED)[0:3,:]
        elif type == self.JacobianType.MIDDLE:
            return pinocchio.computeFrameJacobian(self.model, self.data, self.q, self.end_effector_indeces['middle'], pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED)[0:3,:]
        elif type == self.JacobianType.RING:
            return pinocchio.computeFrameJacobian(self.model, self.data, self.q, self.end_effector_indeces['ring'], pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED)[0:3,:]
        elif type == self.JacobianType.THUMB:
            return pinocchio.computeFrameJacobian(self.model, self.data, self.q, self.end_effector_indeces['thumb'], pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED)[0:3,:]

        J_i = pinocchio.computeFrameJacobian(self.model, self.data, self.q, self.end_effector_indeces['index'], pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        J_ii = pinocchio.computeFrameJacobian(self.model, self.data, self.q, self.end_effector_indeces['middle'], pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        J_iii = pinocchio.computeFrameJacobian(self.model, self.data, self.q, self.end_effector_indeces['ring'], pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        J_iv = pinocchio.computeFrameJacobian(self.model, self.data, self.q, self.end_effector_indeces['thumb'], pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED)

        J = np.vstack([J_i, J_ii, J_iii, J_iv])
        return J

    """
    ---------------------------------------------------------------------------
    Dynamics
    ---------------------------------------------------------------------------
    M ddq + b = tau = M(q)ddq + C(q,dq)dq + G = tau
    """
    def getMassMatrix(self):
        M = pinocchio.crba(self.model, self.data, self.q)
        return M
    def getDynamicDrift(self):
        b = pinocchio.rnea(self.model, self.data, self.q, self.dq, self.ddq)
        return b
    def getGravityVector(self):
        return  pinocchio.computeGeneralizedGravity(self.model, self.data, self.q)
    """
    ---------------------------------------------------------------------------
    Utils
    ---------------------------------------------------------------------------
    """
    def getRandomConfig(self):
        return pinocchio.randomConfiguration(self.model)

    """
    ---------------------------------------------------------------------------
    Static
    ---------------------------------------------------------------------------
    """
    #def evaluateMassMatrix(self, q)