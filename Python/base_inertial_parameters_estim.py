"""Estimate reduced inertial parameters from ROS2 bag joint data.

This script reads joint states from a rosbag, filters the signals,
computes the dynamic regressor for a reduced parameter set, and
estimates parameters by least-squares. It also computes simple
friction terms (viscous + smooth static) and plots results.

The code relies on helper functions and classes from `utils`.
"""

from utils import *
import numpy as np
import pandas as pd
import os

from scipy.signal import butter, filtfilt, savgol_filter

import rosbag2_py
from rclpy.serialization import serialize_message, deserialize_message
from sensor_msgs.msg import JointState


# Fix random seed for any stochastic components (reproducibility)
np.random.seed(41)

# C++ export filename for reduced beta
#CPP_FILENAME = "./ahand_finger_beta.h"
CPP_FILENAME = "/home/leo/Desktop/allegro_hand_ros2_ws/workdir/tmp_ahand_finger_beta.h"
CPP_EXPORT = False

#BAG_PATH =  '/home/leo/Desktop/allegro_hand_ros2_ws/workdir/allegro_hand_4traject_30s_bag'
##BAG_PATH =  '/home/leo/Desktop/allegro_hand_ros2_ws/workdir/allegro_hand_my_torque_bag'
##BAG_PATH = '/home/leo/Desktop/allegro_hand_ros2_ws/workdir/allegro_hand_4traject_bag'
##BAG_PATH = '/home/leo/Desktop/allegro_hand_ros2_ws/workdir/allegro_hand_test_traject_manual_bag'
##BAG_PATH = '/home/leo/Desktop/allegro_hand_ros2_ws/workdir/allegro_hand_test_traject_bag'
##BAG_PATH = '/home/leo/Desktop/allegro_hand_ros2_ws/workdir/allegro_hand_sim_bag'
BAG_PATH =  '/home/leo/Desktop/allegro_hand_ros2_ws/workdir/allegro_hand_thumb_2traject_bag'

#JOINT_NAMES = ['joint_4_0', 'joint_5_0', 'joint_6_0', 'joint_7_0']
JOINT_NAMES = ['joint_12_0', 'joint_13_0', 'joint_14_0', 'joint_15_0']  # Thumb joints

# MASSES_URDF = [0.0176, 0.086, 0.0367, 0.0057] # [Kg]
# INERTIAS_URDF = [[1.32064e-05, 1.22498e-05, 2.64678e-06],
#                  [5.66968e-05, 4.87935e-05, 1.75097e-05],
#                  [1.45504e-05, 1.30076e-05, 3.70193e-06],
#                  [5.27679e-06, 4.04675e-06, 2.638e-06]] # [Kg m^2]

# When True, include rotor inertia terms (I_r * ddq) as separate regressors
ADD_ROTOR_INERTIA = True

# =========================================================================================================================== Parameter reduction
# Parameter reduction: load the robot configuration and compute
# the regrouping matrix `beta` that maps full inertial parameters
# to a reduced parameter vector. We then store its pseudoinverse
# (`beta_pinv`) for projecting the full regressor into reduced space.

# A) Finger
# CONF_FILENAME = os.path.join(os.path.dirname(__file__), "./ahand_finger_generatedFiles/ahand_finger_conf.yaml")
# DH = { 'joints':['R','R','R','R'],

#        'a':     [0,         0,      0.0431,  0.038],
#        'alpha': [0,        np.pi/2,  0,      0],
#        'd':     [0.017,      0,      0,      0],
#        'theta': [np.pi,   np.pi/2,  0,      0]}
# ahand_finger = thunder_ahand_finger()
# ahand_finger.load_conf(CONF_FILENAME)
# _,beta =regrouping(DH, ahand_finger, ADD_ROTOR_INERTIA)
# beta_pinv = np.linalg.pinv(beta)

# B) Thumb
# ==========================================================================================================================
CONF_FILENAME = os.path.join(os.path.dirname(__file__), "./config/thunder/ahand_thumb_conf.yaml")
DH = { 'joints':['R','R','R','R'],

       'a':     [0,         0,      0,        0.038],
       'alpha': [np.pi,  -np.pi/2,  np.pi/2,    0],
       'd':     [0,      0.069,     0,          0],
       'theta': [np.pi,   -np.pi/2,        np.pi/2,    0]}

# Instantiate model object for the thumb and compute regrouping
ahand_finger = thunder_ahand_thumb()
ahand_finger.load_conf(CONF_FILENAME)
print(ahand_finger.get_T_0_0())
exit()
_,beta = regrouping(DH, ahand_finger, ADD_ROTOR_INERTIA)
beta_pinv = np.linalg.pinv(beta)


# =========================================================================================================================== Process data
# ==========================================================================================================================
# Process data: read rosbag, trim to the selected interval and extract
# joint positions, velocities and commanded efforts (torques).
INTERVAL = [5, 10000]
df = read_bag(BAG_PATH, topic_name='/allegroHand_0/joint_states', joint_names=JOINT_NAMES, store=False)

# Align timestamps and select the time window of interest
t0 = df['t'].to_numpy()[0]
df = df[(df['t'] - t0 >= INTERVAL[0]) & (df['t'] - t0 <= INTERVAL[1])]
t_log = df['t'].to_numpy() - t0

# Extract signals (columns named pos_*, vel_*, eff_*) into numpy arrays
q_log = df[[col for col in df.columns if col.startswith('pos_')]].to_numpy()
qd_log = df[[col for col in df.columns if col.startswith('vel_')]].to_numpy()
tau_log = df[[col for col in df.columns if col.startswith('eff_')]].to_numpy()

# Sampling info and backups of original (pre-filter) signals
Dt = np.mean(np.diff(t_log))    # sampling period [s]
F_s = 1.0 / Dt                  # sampling frequency [Hz]
t_ori = t_log.copy()
q_ori = q_log.copy()
qd_ori = qd_log.copy()
tau_ori = tau_log.copy()

# Design Butterworth filter
# ==========================================================================================================================
# Filter design and numerical differentiation
order = 4
# cutoff_freq in Hz (note original comment: should be ~1.6-6.4 Hz)
cutoff_freq = 30/(2*np.pi)
b, a = butter(order, cutoff_freq / (0.5 * F_s), btype='low')

# 1) Zero-phase low-pass filter positions to reduce noise before differentiation
q_filt = filtfilt(b, a, q_log, axis=0)
q_log = q_filt

# 2) Estimate velocity by centered finite differences on filtered positions
dt_centered = t_log[2:] - t_log[:-2]
qd_filt = np.zeros_like(q_log)
qd_filt[1:-1,:] = (q_log[2:,:] - q_log[:-2,:])/(2*dt_centered[:,None])
qd_log = qd_filt

# 3) Estimate acceleration by centered differences on velocities
qdd_filt = np.zeros_like(qd_log)
qdd_filt[1:-1,:] = (qd_log[2:,:] - qd_log[:-2,:])/(2*dt_centered[:,None])
qdd_log = qdd_filt

# 4) Filter commanded torques/efforts with same filter (zero-phase)
tau_filt = filtfilt(b, a, tau_log, axis=0)
tau_log = tau_filt


# =========================================================================================================================== Compute regressor
# ==========================================================================================================================
# Compute the (reduced) regressor for each time sample and augment with
# friction terms. For each timestamp we:
#  - set the model state (q, dq, ddq)
#  - compute the full regressor Y (n_joints x n_params_full)
#  - optionally append rotor inertia columns (diagonal of ddq)
#  - project to reduced regressor Y_hat = Y * beta_pinv
#  - append viscous and smooth static friction columns
Y_log = []
for i in range(t_log.shape[0]):
    q = q_filt[i,:]
    dq = qd_filt[i,:]
    ddq = qdd_filt[i,:]
    ahand_finger.set_q(q)
    ahand_finger.set_dq(dq)
    ahand_finger.set_dqr(dq)
    ahand_finger.set_ddqr(ddq)

    # Full inverse-dynamics regressor (per-joint rows)
    Y = ahand_finger.get_Yr()
    if ADD_ROTOR_INERTIA:
        Y_Ir = np.diag(ddq)
        Y = np.hstack([Y, Y_Ir])

    # Project into reduced-parameter space
    Y_hat = Y @ beta_pinv

    # Friction: viscous damping proportional to dq, and smooth sign for static
    Y_d = np.diag(dq)
    #Y_s = np.diag(2/(1+np.exp(-200*dq)) - 1)
    Y_s = np.diag(np.sign(dq))

    Y_log.append(np.hstack([Y_hat, Y_d, Y_s]))


print("> Computing Dynamics param")
M = len(t_log)
n = ahand_finger.get_numJoints()
print("Number of samples (M): ", M)

# Stack regressors and torques into the big LS problem
YY = np.concatenate([np.asarray(Y) for Y in Y_log], axis=0)  # (M*n) x p
TTau = np.concatenate([np.asarray(t).reshape(n, 1) for t in tau_log], axis=0)  # (M*n) x 1

p = Y_log[0].shape[1]
par_per_link = int(p/n)
print("Shape YY (Mnxp): ", YY.shape)
print("Shape TTau (Mnx1): ", TTau.shape)


# Method 1 - Pinv
print("Computing reduced pi solution by pseudoinverse")
hat_Pi = np.linalg.pinv(YY) @ TTau

# Compute residual norm and singular values for analysis
residual = np.linalg.norm(TTau - YY @ hat_Pi)
print("Residuals (Nm):" )
print(residual)
s = np.linalg.svd(YY, compute_uv=False)

if np.linalg.matrix_rank(YY)<YY.shape[1]:
    print(f"WARNING: Matrix is not full colum rank {np.linalg.matrix_rank(YY)}/{YY.shape[1]} - the solution is not unique" )
else:
    sigma_w = residual/(YY.shape[0]-YY.shape[1])
    C_w = sigma_w * np.linalg.inv(YY.transpose() @ YY)
    sigma_pi = np.sqrt(np.diag(C_w)).reshape(hat_Pi.shape)
    sigma_pi_perc = 100*sigma_pi/np.abs(hat_Pi)

    print("Parameters std. deviation")
    print(np.array2string(sigma_pi[ 0: 1].transpose()))
    print(np.array2string(sigma_pi[ 1: 8].transpose()))
    print(np.array2string(sigma_pi[ 8:16].transpose()))
    print(np.array2string(sigma_pi[16:24].transpose()))
    print(np.array2string(sigma_pi[24:].transpose()))

    print("Parameters quality index (%)")
    print(sigma_pi_perc[ 0: 1].transpose().astype(int))
    print(sigma_pi_perc[ 1: 8].transpose().astype(int))
    print(sigma_pi_perc[ 8:16].transpose().astype(int))
    print(sigma_pi_perc[16:24].transpose().astype(int))
    print(sigma_pi_perc[24:].transpose().astype(int))

    print("Trajectory quality indeces (1 for optimal)")
    print(s)
    print(f"C1 = {np.max(s)/np.min(s)}")
    print(f"C2 = {np.max(s)/np.min(s) + 1*np.abs(np.max(YY)/np.min(YY))}")
    print(f"C3 = {np.max(s)/np.min(s) + 1/np.min(s)}")

    if CPP_EXPORT:
        print(f"Exporting Reduced Parameters in {CPP_FILENAME}...")
        save_beta(CPP_FILENAME, beta, hat_Pi)


# =========================================================================================================================== Plot REDUCED
# ==========================================================================================================================
# Compute residuals (measured torque - predicted torque) for each sample
delta_tau = []
for i in range(len(t_log)):
    q = q_filt[i,:]
    dq = qd_filt[i,:]
    ddq = qdd_filt[i,:]

    ahand_finger.set_q(q)
    ahand_finger.set_dq(dq)
    ahand_finger.set_dqr(dq)
    ahand_finger.set_ddqr(ddq)

    Y = ahand_finger.get_Yr()
    if ADD_ROTOR_INERTIA:
        Y_Ir = np.diag(ddq)
        Y = np.hstack([Y, Y_Ir])
    Y_hat = Y @ beta_pinv

    # friction columns (same construction as in training regressor)
    Y_d = np.diag(dq.flatten())
    #Y_s = np.diag(2/(1+np.exp(-200*dq)) - 1)
    Y_s = np.diag(np.sign(dq))

    Y = np.hstack([Y_hat, Y_d, Y_s])

    tau_est = np.array(Y @ hat_Pi).flatten()
    delta_tau.append(tau_filt[i,:] - tau_est)
delta_tau = np.array(delta_tau)


rmse = np.sqrt(np.mean(np.sum(delta_tau**2, axis=1))) # RMSE = Sqrt{ 1/M Sum{||e||_2}  }
print(f"RMSE: {rmse} Nm" )

# --- 1. Plot joint trajectories ---
plt.figure(figsize=(12,8))
for i in range(4):
    plt.subplot(2,2,i+1)
    plt.plot(t_log, q_log[:,i], 'r', label='q')
    plt.title(f'Position {JOINT_NAMES[i]}')
    plt.grid(True)
    if i==0:
        plt.legend()
plt.tight_layout()
plt.show(block=False)
plt.figure(figsize=(12,8))
for i in range(4):
    plt.subplot(2,2,i+1)
    plt.plot(t_log, qd_log[:,i], 'r', label='dq')
    plt.title(f'Velocity {JOINT_NAMES[i]}')
    plt.grid(True)
    if i==0:
        plt.legend()
plt.tight_layout()
plt.show(block=False)
plt.figure(figsize=(12,8))
for i in range(4):
    plt.subplot(2,2,i+1)
    plt.plot(t_log, qdd_filt[:,i], 'r', label='ddq')
    plt.title(f'Acceleration {JOINT_NAMES[i]}')
    plt.grid(True)
    if i==0:
        plt.legend()
plt.tight_layout()
plt.show(block=False)


# --- 1. Plot tau ---
plt.figure(figsize=(12,8))
for i in range(4):
    plt.subplot(2,2,i+1)
    #plt.plot(t_log, tau_ori[:,i], label = 'tau')
    plt.plot(t_log, tau_log[:,i], 'r', label = 'tau_filtered')
    plt.plot(t_log, tau_log[:,i] - delta_tau[:,i], linestyle='dotted', label = 'hat_tau')
    plt.xlabel('Time [s]')
    plt.ylabel('Torque [Nm]')
    plt.title(f'Torque {JOINT_NAMES[i]}')
    plt.grid(True)
    if i==0:
        plt.legend()
plt.tight_layout()
#plt.show(block=False)

plt.show()
exit()
# =========================================================================================================================== Plot NOT REDUCED
# ==========================================================================================================================
# Compute residuals (measured torque - predicted torque) for each sample
#CONF_FILENAME = os.path.join(os.path.dirname(__file__), "./config/thunder/ahand_middle_conf.yaml")
CONF_FILENAME = os.path.join(os.path.dirname(__file__), "./config/thunder/ahand_thumb_conf.yaml")

# Instantiate model object for the thumb and compute regrouping
ahand_finger = thunder_ahand_finger()
#ahand_finger = thunder_ahand_thumb()

# hat_Pi = np.array([0.005826373940058842,
#         0.001229783398293674,
#         0.001410220625744967,
#         -0.001283322770670198,
#         0.001126186364022898,
#         7.931473718821497e-05,
#         0.0002315044728782006,
#         0.0008531946752300644,
#         0.00184890592760831,
#         -0.0001144619853704485,
#         0.002304931104338838,
#         0.001567645155249121,
#         -0.0001040912801342329,
#         -9.838084330324119e-06,
#         0.0001898529797633629,
#         9.950847010626122e-05,
#         0.0009838889358258327,
#         -6.29592369920291e-05,
#         -0.002161429937376294,
#         -0.0001902243318239401,
#         -4.294515022289427e-05,
#         -4.915571752745692e-05,
#         -3.88067981234312e-05,
#         0.0001626433467579268,
#         -0.09490900224324487,
#         0.005797030735277816,
#         0.005294997543807801,
#         0.01654182480861388,
#         0.01784648227629518,
#         0.01958607315128258,
#         0.006974042991620326,
#         0.01144908736676299])

ahand_finger.load_conf(CONF_FILENAME)
hat_Pi_reduced = hat_Pi.copy()
hat_Pi = ahand_finger.get_par_REG()

delta_tau_reduced = []
delta_tau = []
for i in range(len(t_log)):
    q = q_filt[i,:]
    dq = qd_filt[i,:]
    ddq = qdd_filt[i,:]

    ahand_finger.set_q(q)
    ahand_finger.set_dq(dq)
    ahand_finger.set_dqr(dq)
    ahand_finger.set_ddqr(ddq)

    Y = ahand_finger.get_Yr()
    Y_reduced = np.hstack([Y, np.diag(ddq)]) @ beta_pinv

    tau_est = np.array(Y @ hat_Pi).flatten()
    tau_est_reduced = np.array(Y_reduced @ hat_Pi_reduced[:24]).flatten()
    delta_tau.append(tau_filt[i,:] - tau_est)
    delta_tau_reduced.append(tau_filt[i,:] - tau_est_reduced)
delta_tau = np.array(delta_tau)
delta_tau_reduced = np.array(delta_tau_reduced)

# --- 1. Plot tau ---
plt.figure(figsize=(12,8))
for i in range(4):
    plt.subplot(2,2,i+1)
    #plt.plot(t_log, tau_ori[:,i], label = 'tau')
    plt.plot(t_log, tau_log[:,i], 'r', label = 'tau_filtered')
    plt.plot(t_log, tau_log[:,i] - delta_tau[:,i], linestyle='dotted', label = 'hat_tau')
    plt.plot(t_log, tau_log[:,i] - delta_tau_reduced[:,i], linestyle='dotted', label = 'hat_tau_reduced')
    plt.xlabel('Time [s]')
    plt.ylabel('Torque [Nm]')
    plt.title(f'Torque {JOINT_NAMES[i]}')
    plt.grid(True)
    if i==0:
        plt.legend()
plt.tight_layout()
plt.show()