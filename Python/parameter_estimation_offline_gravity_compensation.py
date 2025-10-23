import time
import os
import matplotlib.pyplot as plt
from scipy.linalg import solve_continuous_lyapunov, block_diag
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState
import casadi

import numpy as np
from tqdm import tqdm

import pandas as pd

from scipy.signal import butter, filtfilt, savgol_filter

import sys
sys.path.append("/home/leo/thunder_dynamics/ahand_finger_generatedFiles/build") # Where the .so file is located. 
sys.path.append("/home/leo/thunder_dynamics/franka_as_generatedFiles/build") # Where the .so file is located. 
# Note: This is not needed if the .so file is in the same directory as the python script
from thunder_ahand_finger_py import thunder_ahand_finger
from thunder_franka_as_py import thunder_franka_as

MESH_DIR = './description/urdf/meshes/'
URDF_PATH = './description/urdf/allegro_hand_description_right_B.urdf'
XML_PATH = './description/urdf/allegro_hand_description_right_B.xml'
BAG_PATH = '/home/leo/Desktop/allegro_hand_ros2_ws/workdir/allegro_hand_bag'
JOINT_NAMES = ['joint_4_0', 'joint_5_0', 'joint_6_0', 'joint_7_0']
SAVE_FILENAME = '/home/leo/Desktop/ahand_dynamic_gravity_compensation.txt'

def read_bag(file_path, store=False, path_out=None):
    first = True
    # Open the bag reader
    reader = rosbag2_py.SequentialReader()

    # set options and open the bag
    storage_options = rosbag2_py.StorageOptions(uri=file_path)
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader.open(storage_options, converter_options)

    dataset = []
    header = []
    # scroll the bag
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        if topic == '/allegroHand_0/joint_states':
            msg = deserialize_message(data, JointState)
            joint_names = msg.name

            # select specific joints
            indices = [joint_names.index(name) for name in JOINT_NAMES if name in joint_names]
            selected_joint_names = [joint_names[i] for i in indices]
            if first:
                joint_names = msg.name
                # get header
                pos_header    = [f"pos_{name}" for name in selected_joint_names]
                effort_header = [f"eff_{name}" for name in selected_joint_names]
                #vel_header    = [f"vel_{name}" for name in selected_joint_names]
                #acc_header    = [f"acc_{name}" for name in selected_joint_names]
                #header = ['t'] + pos_header + vel_header + acc_header + effort_header
                header = ['t'] + pos_header + effort_header
                first = False
            elif joint_names != msg.name:
                raise ValueError("Joint names do not match")

            # Store data
            selected_positions = [msg.position[i] for i in indices]
            selected_effort = [msg.effort[i] for i in indices]
            row = [t/1e9] + selected_positions + selected_effort
            dataset.append(row)

    # store in dataframe
    df = pd.DataFrame(dataset, columns=header)
    if store:
        df.to_csv(file_path, index=False)

    return df

# 3X3 MATRICES EIGENVALUES COMPUTATION  VIA POWER ITERATION METHOD
def powerm(v, A, niter): #POWER ITERATION METHOD
    for i in range(niter):
        v = A @ v
        v = v/casadi.norm_2(v)
    l = v.T@A@v
    return v, l
def def_powerm(a, niter=None): #DEFLATION VERSION OF PIM FOR 3X3 MATRIX ---> CALCULATE ALL EIGENVALUES FROM GREATER TO SMALLER
    #AND THEIR EIGENVECTORS
    v0 = casadi.DM([1,1,1])

    if niter is None:
        niter = 40

    v1,l1 = powerm(v0, a, niter)

    v0 = casadi.DM([0,1,1])
    a2 = a - l1*(v1 @ v1.T)
    v2, l2 = powerm(v0, a2, niter)

    v0 = casadi.DM([0,0,1])
    a3 = a2 - l2 * (v2 @ v2.T)
    v3, l3 = powerm(v0, a3, niter)


    D = casadi.vertcat(l1,l2,l3)
    V = casadi.horzcat(v1,v2,v3)
    return V, D

# Skew-symmetric matrix
def skew(v):
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])


def save_dynamics(filename, hat_Pi, num_joints):
    masses = [(hat_Pi[i*10]).item() for i in range(num_joints)]

    CoMs_x = [(hat_Pi[i*10+1]/masses[i]).item() for i in range(num_joints)]        #pi = m*cx
    CoMs_y = [(hat_Pi[i*10+2]/masses[i]).item() for i in range(num_joints)]
    CoMs_z = [(hat_Pi[i*10+3]/masses[i]).item() for i in range(num_joints)]

    Is_xx = [(hat_Pi[i*10+4] - (masses[i]*CoMs_y[i]**2 - masses[i]*CoMs_z[i]**2)).item()      for i in range(num_joints)]
    Is_xy = [(hat_Pi[i*10+5] - (masses[i]*CoMs_x[i]*CoMs_y[i])).item()                        for i in range(num_joints)]
    Is_xz = [(hat_Pi[i*10+5] - (masses[i]*CoMs_x[i]*CoMs_z[i])).item()                        for i in range(num_joints)]
    Is_yy = [(hat_Pi[i*10+7] - (masses[i]*CoMs_x[i]**2 - masses[i]*CoMs_z[i]**2)).item()   for i in range(num_joints)]
    Is_yz = [(hat_Pi[i*10+8] - (masses[i]*CoMs_y[i]*CoMs_z[i])).item()   for i in range(num_joints)]
    Is_zz = [(hat_Pi[i*10+9] - (masses[i]*CoMs_x[i]**2 - masses[i]*CoMs_y[i]**2)).item()   for i in range(num_joints)]

    with open(filename, 'w') as f:
        f.write("=== Dynamic parameters ======\n")
        f.write(f"PI size: {hat_Pi.shape}\n")
        for i in range(num_joints):
            f.write(f"\tlink {i}:\n")
            f.write(f"\t\tinertial:\n")
            f.write(f"\t\t\tsymb: [1,1,1,1,1,1,1,1,1,1]\n")
            f.write(f"\t\t\tmass: {masses[i]}\n")
            f.write(f"\t\t\tCoM_x: {CoMs_x[i]}\n")
            f.write(f"\t\t\tCoM_y: {CoMs_y[i]}\n")
            f.write(f"\t\t\tCoM_z: {CoMs_z[i]}\n")
            f.write(f"\t\t\tIxx: {Is_xx[i]}\n")
            f.write(f"\t\t\tIxy: {Is_xy[i]}\n")
            f.write(f"\t\t\tIxz: {Is_xz[i]}\n")
            f.write(f"\t\t\tIyy: {Is_yy[i]}\n")
            f.write(f"\t\t\tIyz: {Is_yz[i]}\n")
            f.write(f"\t\t\tIzz: {Is_zz[i]}\n")
        f.write("=============================\n")
def load_dynamics(filename, num_joints): # filename -> (hat_Pi, dict)
    dynamics = {}
    hat_Pi = np.zeros(num_joints)
    with open(filename, 'r') as f:
        lines = f.readlines()

    for line in lines:
        line = line.strip()
        if line.startswith('mass:'):
            dynamics.setdefault('mass', []).append(float(line.split(':')[1]))
        elif line.startswith('CoM_x:'):
            dynamics.setdefault('CoM_x', []).append(float(line.split(':')[1]))
        elif line.startswith('CoM_y:'):
            dynamics.setdefault('CoM_y', []).append(float(line.split(':')[1]))
        elif line.startswith('CoM_z:'):
            dynamics.setdefault('CoM_z', []).append(float(line.split(':')[1]))
        elif line.startswith('Ixx:'):
            dynamics.setdefault('Ixx', []).append(float(line.split(':')[1]))
        elif line.startswith('Ixy:'):
            dynamics.setdefault('Ixy', []).append(float(line.split(':')[1]))
        elif line.startswith('Ixz:'):
            dynamics.setdefault('Ixz', []).append(float(line.split(':')[1]))
        elif line.startswith('Iyy:'):
            dynamics.setdefault('Iyy', []).append(float(line.split(':')[1]))
        elif line.startswith('Iyz:'):
            dynamics.setdefault('Iyz', []).append(float(line.split(':')[1]))
        elif line.startswith('Izz:'):
            dynamics.setdefault('Izz', []).append(float(line.split(':')[1]))

    # reconstruct hat_Pi
    hat_Pi[0::10] = dynamics['mass']
    hat_Pi[1::10] = dynamics['CoM_x'] * dynamics['mass']
    hat_Pi[2::10] = dynamics['CoM_y'] * dynamics['mass']
    hat_Pi[3::10] = dynamics['CoM_z'] * dynamics['mass']
    hat_Pi[4::10] = dynamics['Ixx'] + dynamics['mass'] * (dynamics['CoM_y']**2 + dynamics['CoM_z']**2)
    hat_Pi[5::10] = dynamics['Ixy'] + dynamics['mass'] * (dynamics['CoM_x'] * dynamics['CoM_y'])
    hat_Pi[6::10] = dynamics['Ixz'] + dynamics['mass'] * (dynamics['CoM_x'] * dynamics['CoM_z'])
    hat_Pi[7::10] = dynamics['Iyy'] + dynamics['mass'] * (dynamics['CoM_x']**2 + dynamics['CoM_z']**2)
    hat_Pi[8::10] = dynamics['Iyz'] + dynamics['mass'] * (dynamics['CoM_y'] * dynamics['CoM_z'])
    hat_Pi[9::10] = dynamics['Izz'] + dynamics['mass'] * (dynamics['CoM_x']**2 + dynamics['CoM_y']**2)


    return hat_Pi, dynamics


print("> Loading position and effort arrays from bag")
df = read_bag(BAG_PATH, store=False)
# =========================================================================================================================== Process data
print("> Processing data")
print("\t - Filtering position and estimating velocities")
# Process data
t_log = df['t'].to_numpy()
q_log = df[[col for col in df.columns if col.startswith('pos_')]].to_numpy()
tau_log = df[[col for col in df.columns if col.startswith('eff_')]].to_numpy()
dt = np.mean(np.diff(t_log))    # sampling period
f_s = 1.0 / dt                  # sampling frequency
t_ori = t_log.copy()            # store original data
q_ori = q_log.copy()

# 1. Filtering position
""" Savitzy-golay
# Parameters
window_length = 41   # must be odd, ~ window of ~50 ms if fs=1000Hz
polyorder = 3        # polynomial order (2–4 typical)
# Filter position
q_filt = savgol_filter(q_log, window_length=window_length, polyorder=polyorder, axis=0)
# First derivative → velocity
qd_log = savgol_filter(q_log, window_length=window_length, polyorder=polyorder, deriv=1, delta=dt, axis=0)
# Second derivative → acceleration
qdd_log = savgol_filter(q_log, window_length=window_length, polyorder=polyorder, deriv=2, delta=dt, axis=0)
# store filtered joint position
q_ori = q_log
q_log = q_filt
"""


"""# moving average filter + finite difference
window_size = 50
# 1. filter position
q_filt = np.convolve(q_log[:,0], np.ones(window_size)/window_size, mode='same').reshape(-1,1)
for i in range(1,q_log.shape[1]):
    # filtering each joint separately
    q_filt_i = np.convolve(q_log[:,i], np.ones(window_size)/window_size, mode='same').reshape(-1,1)
    q_filt = np.hstack((q_filt, q_filt_i))

# discard edges
edge = window_size // 2
q_log = q_filt[edge:-edge,:]
t_log = t_log[edge:-edge]

# 2. First derivative → velocity
qd = np.gradient(q_log, dt, axis=0)
t_vel = t_log.copy()
# filter velocity
qd_filt = np.convolve(qd[:,0], np.ones(window_size)/window_size, mode='same').reshape(-1,1)
for i in range(1, qd.shape[1]):
    # filtering each joint separately
    qd_filt_i = np.convolve(qd[:,i], np.ones(window_size)/window_size, mode='same').reshape(-1,1)
    qd_filt =  np.hstack((qd_filt, qd_filt_i))
# discard edges
t_log = t_log[edge:-edge]
q_log = q_log[edge:-edge,:]
qd_log = qd_filt[edge:-edge,:]

# 3. Second derivative → acceleration
qdd = np.gradient(qd_log, dt, axis=0)
t_acc = t_log.copy()
# filter accedeleration
qdd_filt = np.convolve(qdd[:,0], np.ones(window_size)/window_size, mode='same').reshape(-1,1)
for i in range(1,qd.shape[1]):
    # filtering each joint separately
    qdd_filt_i = np.convolve(qdd[:,i], np.ones(window_size)/window_size, mode='same').reshape(-1,1)
    qdd_filt =  np.hstack((qdd_filt, qdd_filt_i))
# discard edges
t_log = t_log[edge:-edge]
q_log = q_log[edge:-edge,:]
qd_log = qd_log[edge:-edge,:]
qdd_log = qdd_filt[edge:-edge,:]"""

# Butterworth filter + finite difference
# Design Butterworth filter
order = 4
cutoff_freq = 1.0  # Hz
b, a = butter(order, cutoff_freq / (0.5 * f_s), btype='low')
# 1. Filter position
q_filt = filtfilt(b, a, q_log, axis=0)
q_log = q_filt
# 2. First derivative → velocity
qd = np.gradient(q_log, dt, axis=0)
qd_filt = filtfilt(b, a, qd, axis=0)
qd_log = qd_filt
t_vel = t_log.copy()
# 3. Second derivative → acceleration
qdd = np.gradient(qd_log, dt, axis=0)
qdd_filt = filtfilt(b, a, qdd, axis=0)
qdd_log = qdd_filt
t_acc = t_log.copy()


## --- 1. Plot joint trajectories ---
#plt.figure(figsize=(12,8))
#for i in range(4):
#    plt.subplot(2,2,i+1)
#    plt.plot(t_ori, q_ori[:,i], 'b', label='q')# Plot joint limits
#    plt.plot(t_log, q_log[:,i], 'r', label='q_filt')# Plot joint limits
#    plt.title(f'Joint {JOINT_NAMES[i]}')
#    plt.grid(True)
#    if i==0:
#        plt.legend()
#plt.tight_layout()
#plt.show(block=False)
## --- 1. Plot joint trajectories ---
#plt.figure(figsize=(12,8))
#for i in range(4):
#    plt.subplot(2,2,i+1)
#    plt.plot(t_vel, qd[:,i], 'b', label='dq')# Plot joint limits
#    plt.plot(t_log, qd_log[:,i], 'r', label='dq_filt')# Plot joint limits
#    plt.title(f'Joint {JOINT_NAMES[i]}')
#    plt.grid(True)
#    if i==0:
#        plt.legend()
#plt.tight_layout()
#plt.show(block=False)
#plt.figure(figsize=(12,8))
#for i in range(4):
#    plt.subplot(2,2,i+1)
#    plt.plot(t_acc, qdd[:,i], 'b', label='ddq')# Plot joint limits
#    plt.plot(t_log, qdd_log[:,i], 'r', label='ddq_filt')# Plot joint limits
#    plt.title(f'Joint {JOINT_NAMES[i]}')
#    plt.grid(True)
#    if i==0:
#        plt.legend()
#plt.tight_layout()
#plt.show()
#
#exit(0)


print("\t - Computing Regressor")
thunder_ahand = thunder_ahand_finger()
thunder_ahand.load_conf("/home/leo/thunder_dynamics/ahand_finger_generatedFiles/ahand_finger_conf.yaml")

Y_log = []
for i in range(t_log.shape[0]):
    thunder_ahand.set_q(q_log[i])
    thunder_ahand.set_dq(qd_log[i])
    thunder_ahand.set_dqr(qd_log[i])
    thunder_ahand.set_ddqr(qdd_log[i])
    Y = thunder_ahand.get_reg_G()
    Y_log.append(Y)


print("> Computing Dynamics param")
M = len(t_log)
n = thunder_ahand.get_numJoints()
print("Number of samples (M): ", M)
# YY @ Pi = TTau, where YY (is Mnxp), TTau (is Mnx1) and Pi (is px1)
#YY = np.array(Y_log[0])
#TTau = np.array(tau_log[0]).reshape(n,1)
#for i in tqdm(range(1, len(t_log)), desc="stacking regressor"):
#    YY = np.vstack((YY, 
#                    np.array(Y_log[i])))
#    TTau = np.vstack((TTau, 
#                      np.array(tau_log[i]).reshape(n, 1)))
YY = np.concatenate([np.asarray(Y) for Y in Y_log], axis=0)
TTau = np.concatenate([np.asarray(t).reshape(n, 1) for t in tau_log], axis=0)


p = Y_log[0].shape[1]
print("Shape YY (Mnxp): ", YY.shape)
print("Shape TTau (Mnx1): ", TTau.shape)

# Method 1 - Pinv
print("Computing initial guess by pseudoinverse")
hat_Pi = np.linalg.pinv(YY)@TTau



print("Defining costrained minimization problem")
positive_th = 0.0001
## Method 2 - Least Squares
hat_Pi_sym = casadi.SX.sym('Pi', p, 1)    # Parameters to estimate
g = []          # Constraints
ub = []         # Upper bound
lb = []         # Lower bound

for i in range(thunder_ahand.get_numJoints()):
    # Mass must be positive
    g.append(hat_Pi_sym[i*10])
    lb.append(positive_th)
    ub.append(0.5)             # [Kg]

    # CoM inside a spere of radius r
    cx = hat_Pi_sym[i*10 + 1]/hat_Pi_sym[i*10]
    cy = hat_Pi_sym[i*10 + 2]/hat_Pi_sym[i*10]
    cz = hat_Pi_sym[i*10 + 3]/hat_Pi_sym[i*10]
    c = np.array([cx,cy,cz])

    g.append(np.sqrt(c@c.T))
    lb.append(0)
    ub.append(0.1)              # [m]

    # Semi-definite inertia matrix
    Jxx = hat_Pi_sym[i*10 + 4]
    Jxy = hat_Pi_sym[i*10 + 5]
    Jxz = hat_Pi_sym[i*10 + 6]
    Jyy = hat_Pi_sym[i*10 + 7]
    Jyz = hat_Pi_sym[i*10 + 8]
    Jzz = hat_Pi_sym[i*10 + 9]
    J = np.array([[Jxx, Jxy, Jxz],
                   [Jxy, Jyy, Jyz],
                   [Jxz, Jyz, Jzz]])
    Ib = J - hat_Pi_sym[i*10] * skew(c)@skew(c).T

    _,l = def_powerm(Ib, niter=40)
    g.append((Ib[0,0]+Ib[1,1]+Ib[2,2])/2 - l[0])

    lb.append(positive_th)
    ub.append(np.inf)


prob = {}
prob['x'] = hat_Pi_sym
prob['f'] = (YY@hat_Pi_sym - TTau).T@(YY@hat_Pi_sym - TTau)
prob['g'] = casadi.vertcat(*g)
solver = casadi.nlpsol('sol', 'ipopt', prob)

hat_Pi_0 = np.random.rand(p,1)
sol = solver(x0=hat_Pi_0, lbg=lb,ubg=ub)
hat_Pi = sol['x'].full()

print("Residual norm: ", np.linalg.norm(YY@hat_Pi - TTau))
print("> Extracting dinamic parameters")

masses = [(hat_Pi[i*10]).item() for i in range(thunder_ahand.get_numJoints())]

CoMs_x = [(hat_Pi[i*10+1]/masses[i]).item() for i in range(thunder_ahand.get_numJoints())]        #pi = m*cx
CoMs_y = [(hat_Pi[i*10+2]/masses[i]).item() for i in range(thunder_ahand.get_numJoints())]
CoMs_z = [(hat_Pi[i*10+3]/masses[i]).item() for i in range(thunder_ahand.get_numJoints())]

Is_xx = [(hat_Pi[i*10+4] - (masses[i]*CoMs_y[i]**2 - masses[i]*CoMs_z[i]**2)).item()        for i in range(thunder_ahand.get_numJoints())]
Is_xy = [(hat_Pi[i*10+5] - (masses[i]*CoMs_x[i]*CoMs_y[i])).item()                          for i in range(thunder_ahand.get_numJoints())]
Is_xz = [(hat_Pi[i*10+5] - (masses[i]*CoMs_x[i]*CoMs_z[i])).item()                          for i in range(thunder_ahand.get_numJoints())]
Is_yy = [(hat_Pi[i*10+7] - (masses[i]*CoMs_x[i]**2 - masses[i]*CoMs_z[i]**2)).item()        for i in range(thunder_ahand.get_numJoints())]
Is_yz = [(hat_Pi[i*10+8] - (masses[i]*CoMs_y[i]*CoMs_z[i])).item()                          for i in range(thunder_ahand.get_numJoints())]
Is_zz = [(hat_Pi[i*10+9] - (masses[i]*CoMs_x[i]**2 - masses[i]*CoMs_y[i]**2)).item()        for i in range(thunder_ahand.get_numJoints())]

print("=== Dynamic parameters ======")
print("PI size:", hat_Pi.shape)
for i in range(thunder_ahand.get_numJoints()):
    print(f"\tlink {i}:")
    print(f"\t\tinertial:")
    print(f"\t\t\tsymb: [1,1,1,1,1,1,1,1,1,1]")
    print(f"\t\t\tmass: {masses[i]}")
    print(f"\t\t\tCoM_x: {CoMs_x[i]}")
    print(f"\t\t\tCoM_y: {CoMs_y[i]}")
    print(f"\t\t\tCoM_z: {CoMs_z[i]}")
    print(f"\t\t\tIxx: {Is_xx[i]}")
    print(f"\t\t\tIxy: {Is_xy[i]}")
    print(f"\t\t\tIxz: {Is_xz[i]}")
    print(f"\t\t\tIyx: {Is_yy[i]}")
    print(f"\t\t\tIyy: {Is_yz[i]}")
    print(f"\t\t\tIzz: {Is_zz[i]}")
print("=============================")
save_dynamics(SAVE_FILENAME, hat_Pi, thunder_ahand.get_numJoints())

thunder_ahand.set_par_DYN(hat_Pi)
delta_tau = []
for i in range(len(t_log)):
    q =q_log[i,:]
    dq = qd_log[i,:]
    ddq = qdd_log[i,:]
    thunder_ahand.set_q(q)
    thunder_ahand.set_dq(dq)
    thunder_ahand.set_dqr(dq)
    thunder_ahand.set_ddqr(ddq)
    Y = thunder_ahand.get_reg_G()
    tau_est = np.array(Y @ hat_Pi).flatten()
    delta_tau.append(tau_log[i,:] - tau_est)
delta_tau = np.array(delta_tau)

print("> Making Plot")

# --- 1. Plot joint trajectories ---
plt.figure(figsize=(12,8))
for i in range(4):
    plt.subplot(2,2,i+1)
    plt.plot(t_log, q_ori[:,i], 'b', label='q')# Plot joint limits
    plt.plot(t_log, q_log[:,i], 'r', label='q_filt')# Plot joint limits
    #plt.plot(t_log, q_log[:,i] - e_log[:,i], 'b', linestyle='dotted', label='q_ref')# Plot joint desired trajectory
    #plt.hlines(JOINT_LIMITS[joint_indices[i], 0], t_log[0], t_log[-1], colors='k', linestyles='dotted', linewidth=1)
    #plt.hlines(JOINT_LIMITS[joint_indices[i], 1], t_log[0], t_log[-1], colors='k', linestyles='dotted', linewidth=1)
    plt.title(f'Joint {JOINT_NAMES[i]}')
    plt.grid(True)
    if i==0:
        plt.legend()
plt.tight_layout()
plt.show(block=False)


# --- 3. Plot torques ---
plt.figure(figsize=(12,8))
for i in range(4):
    plt.subplot(2,2,i+1)
    plt.plot(t_log, tau_log[:,i], label = 'tau')
    plt.plot(t_log, tau_log[:,i]-delta_tau[:,i], linestyle='dotted', label = 'hat_tau')
    plt.xlabel('Time [s]')
    plt.ylabel('Torque [Nm]')
    plt.title(f'Joint {JOINT_NAMES[i]}')
    plt.grid(True)
    if i==0:
        plt.legend()
plt.tight_layout()
plt.show(block=False)

# --- 3. Plot delta torques ---
plt.figure(figsize=(12,8))
for i in range(4):
    plt.subplot(2,2,i+1)
    plt.plot(t_log, delta_tau[:,i], label = 'delta_tau')
    plt.xlabel('Time [s]')
    plt.ylabel('Torque [Nm]')
    plt.title(f'Joint {JOINT_NAMES[i]}')
    plt.grid(True)
    if i==0:
        plt.legend()
plt.tight_layout()
plt.show()

#exit(0)