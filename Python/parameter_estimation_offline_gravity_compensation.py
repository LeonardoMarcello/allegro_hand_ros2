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
sys.path.append("/home/leo/thunder_dynamics/ahand_finger_generatedFiles_py/build") # Where the .so file is located. 
sys.path.append("/home/leo/thunder_dynamics/franka_as_generatedFiles/build") # Where the .so file is located. 
# Note: This is not needed if the .so file is in the same directory as the python script
from thunder_ahand_finger_py import thunder_ahand_finger

MESH_DIR = './description/urdf/meshes/'
URDF_PATH = './description/urdf/allegro_hand_description_right_B.urdf'
XML_PATH = './description/urdf/allegro_hand_description_right_B.xml'
BAG_PATH = '/home/leo/Desktop/allegro_hand_ros2_ws/workdir/allegro_hand_pb_gc_bag'
JOINT_NAMES = ['joint_4_0', 'joint_5_0', 'joint_6_0', 'joint_7_0']
SAVE_FILENAME = '/home/leo/Desktop/ahand_dynamic_gravity_compensation.txt'

MASSES_URDF = [0.0176, 0.086, 0.0367, 0.0057] # [Kg]
HCK_MASSES = [0.0168, 0.1023, 0.0405, 0.0603] # [Kg]
HCK_KP = [1, 1, 1, 1] # [Nm/rad]
HCK_KD = [0.04, 0.15, 0.04, 0.04] # [Nm/(rad/s^2)]
HCK_D = [0.015*0.004*np.sqrt(80), 0.015*0.008*np.sqrt(90), 0.015*0.008*np.sqrt(90), 0.015*0.004*np.sqrt(80)] # [Nm/(rad/s^2)]

def read_bag(file_path, store=False, path_out=None):
    first = True
    first_topic2 = True
    # Open the bag reader
    reader = rosbag2_py.SequentialReader()

    # set options and open the bag
    storage_options = rosbag2_py.StorageOptions(uri=file_path)
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader.open(storage_options, converter_options)

    dataset = []
    dataset_topic2 = []
    header = []
    header_topic2 = []
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
                vel_header    = [f"vel_{name}" for name in selected_joint_names]
                #acc_header    = [f"acc_{name}" for name in selected_joint_names]
                #header = ['t'] + pos_header + vel_header + acc_header + effort_header
                header = ['t'] + pos_header + effort_header + vel_header
                first = False
            elif joint_names != msg.name:
                raise ValueError("Joint names do not match")

            # Store data
            selected_positions = [msg.position[i] for i in indices]
            selected_effort = [msg.effort[i] for i in indices]
            selected_vel = [msg.velocity[i] for i in indices]
            row = [t/1e9] + selected_positions + selected_effort + selected_vel
            dataset.append(row)
        elif topic == '/allegroHand_0/joint_cmd':
            msg = deserialize_message(data, JointState)
            joint_names = msg.name

            # select specific joints
            indices = [joint_names.index(name) for name in JOINT_NAMES if name in joint_names]
            selected_joint_names = [joint_names[i] for i in indices]
            if first_topic2:
                joint_names = msg.name
                # get header
                pos_des_header    = [f"pos_des_{name}" for name in selected_joint_names]
                vel_des_header    = [f"vel_des_{name}" for name in selected_joint_names]
                header_topic2 = ['t'] + pos_des_header + vel_des_header
                first_topic2 = False
            elif joint_names != msg.name:
                raise ValueError("Joint names do not match")

            # Store data
            selected_positions = [msg.position[i] for i in indices]
            selected_vel = [msg.velocity[i] for i in indices]
            row = [t/1e9] + selected_positions + selected_vel
            dataset_topic2.append(row)
    # store in dataframe
    df = pd.DataFrame(dataset, columns=header)
    if store:
        df.to_csv(file_path, index=False)

    # store eventua topic 2
    if not(first_topic2):
        df2 = pd.DataFrame(dataset_topic2, columns=header_topic2)
        if store:
            df.to_csv(file_path, index=False)
        return (df,df2)

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
df, df2 = read_bag(BAG_PATH, store=False)
# =========================================================================================================================== Process data
print("> Processing data")
print("\t - Filtering position and estimating velocities")
# Process data
t_log = df['t'].to_numpy()
t0 = t_log[0]
t_log = t_log - t0
q_log = df[[col for col in df.columns if col.startswith('pos_')]].to_numpy()
tau_log = df[[col for col in df.columns if col.startswith('eff_')]].to_numpy()
qd_log = df[[col for col in df.columns if col.startswith('vel_')]].to_numpy()

t_des_log = df2['t'].to_numpy()
t_des_log = t_des_log - t0
q_des_log = df2[[col for col in df2.columns if col.startswith('pos_des_')]].to_numpy()


tau_log_np = np.zeros_like(tau_log)
for i in range(q_log.shape[0]):
    q = q_log[i]
    qd = qd_log[i]
    tau = tau_log[i]
    idx_des = np.searchsorted(t_des_log, t_log[i], side='right') - 1
    #if idx_des == 0:
    #    idx_des = 1
    if idx_des >= t_des_log.shape[0]:
        idx_des = t_des_log.shape[0]-1

    q_des = q_des_log[idx_des]

    tau_log_np[i,0] = tau[0] - ( HCK_KP[0]*(q_des[0] - q[0]) - HCK_KD[0]*(qd[0]) )
    tau_log_np[i,1] = tau[1] - ( HCK_KP[1]*(q_des[1] - q[1]) - HCK_KD[1]*(qd[1]) )
    tau_log_np[i,2] = tau[2] - ( HCK_KP[2]*(q_des[2] - q[2]) - HCK_KD[2]*(qd[2]) )
    tau_log_np[i,3] = tau[3] - ( HCK_KP[3]*(q_des[3] - q[3]) - HCK_KD[3]*(qd[3]) )
tau_log = tau_log_np.copy()

print("\t - Computing Regressor")
thunder_ahand = thunder_ahand_finger()
thunder_ahand.load_conf("/home/leo/thunder_dynamics/ahand_finger_generatedFiles/ahand_finger_conf.yaml")

Y_log = []
for i in range(t_log.shape[0]):
    thunder_ahand.set_q(q_log[i])
    thunder_ahand.set_dq(qd_log[i])
    thunder_ahand.set_dqr(qd_log[i])
    Y = thunder_ahand.get_reg_G()
    Y = Y[:,[0,1,2,3,  10,11,12,13,  20,21,22,23,  30,31,32,33 ]]   # Neglet inertia 
    # add friction damping and static
    #Y_d = -np.diag(qd_log[i])
    #Y_s = -np.diag(np.tanh(40*qd_log[i]))#np.diag(2/(1+np.exp(-40*qd_log[i])) - 1)
    #Y = np.hstack([Y, Y_d, Y_s])
    Y_log.append(Y)


print("> Computing Dynamics param")
M = len(t_log)
n = thunder_ahand.get_numJoints()
print("Number of samples (M): ", M)
YY = np.concatenate([np.asarray(Y) for Y in Y_log], axis=0)
TTau = np.concatenate([np.asarray(t).reshape(n, 1) for t in tau_log], axis=0)


p = Y_log[0].shape[1]
print("Shape YY (Mnxp): ", YY.shape)
print("Shape TTau (Mnx1): ", TTau.shape)

# Method 1 - Pinv
print("Computing initial guess by pseudoinverse")
hat_Pi = np.linalg.pinv(YY)@TTau



print("Defining costrained minimization problem")
## Method 2 - Least Squares
hat_Pi_sym = casadi.SX.sym('Pi', p, 1)    # Parameters to estimate
g = []          # Constraints
ub = []         # Upper bound
lb = []         # Lower bound

for i in range(thunder_ahand.get_numJoints()):
    # Mass must be positive
    g.append(hat_Pi_sym[i*4])
    #lb.append(0)
    #ub.append(5)  
    lb.append(MASSES_URDF[i])
    ub.append(MASSES_URDF[i])  
    #lb.append(HCK_MASSES_URDF[i])
    #ub.append(HCK_MASSES_URDF[i])             # [Kg]

    # CoM inside a spere of radius r
    cx = hat_Pi_sym[i*4 + 1]/hat_Pi_sym[i*4]
    cy = hat_Pi_sym[i*4 + 2]/hat_Pi_sym[i*4]
    cz = hat_Pi_sym[i*4 + 3]/hat_Pi_sym[i*4]
    c = np.array([cx,cy,cz])

    g.append(np.sqrt(c@c.T))
    lb.append(0)
    ub.append(.3)              # [m]

    # Damping
    #g.append(hat_Pi_sym[16+i])
    #lb.append(HCK_D[i])
    #ub.append(HCK_D[i])
    ## Static
    #g.append(hat_Pi_sym[20+i])
    #lb.append(0.0)
    #ub.append(.0)



prob = {}
prob['x'] = hat_Pi_sym
prob['f'] = (YY@hat_Pi_sym - TTau).T@(YY@hat_Pi_sym - TTau)
prob['g'] = casadi.vertcat(*g)
solver = casadi.nlpsol('sol', 'ipopt', prob)

hat_Pi_0 = 0.0001*np.random.rand(p,1)
sol = solver(x0=hat_Pi_0, lbg=lb,ubg=ub)
hat_Pi = sol['x'].full()

print("Residual norm: ", np.linalg.norm(YY@hat_Pi - TTau))
print("> Extracting dinamic parameters")

masses = [(hat_Pi[i*4]).item() for i in range(thunder_ahand.get_numJoints())]

CoMs_x = [(hat_Pi[i*4+1]/masses[i]).item() for i in range(thunder_ahand.get_numJoints())]        #pi = m*cx
CoMs_y = [(hat_Pi[i*4+2]/masses[i]).item() for i in range(thunder_ahand.get_numJoints())]
CoMs_z = [(hat_Pi[i*4+3]/masses[i]).item() for i in range(thunder_ahand.get_numJoints())]

#Ds = [(hat_Pi[16 + i]).item()        for i in range(n)]
#Fs = [(hat_Pi[20 + i]).item()        for i in range(n)]
print("=== Dynamic parameters ======")
print("PI size:", hat_Pi.shape)
#print("PI: \n", hat_Pi)
for i in range(thunder_ahand.get_numJoints()):
    print(f"\tlink {i+1}:")
    print(f"\t\tinertial:")
    print(f"\t\t\tsymb: [1,1,1,1,1,1,1,1,1,1]")
    print(f"\t\t\tmass: {masses[i]}")
    print(f"\t\t\tCoM_x: {CoMs_x[i]}")
    print(f"\t\t\tCoM_y: {CoMs_y[i]}")
    print(f"\t\t\tCoM_z: {CoMs_z[i]}")
    #print(f"\t\t\tDs: {Ds[i]}")
    #print(f"\t\t\tFs: {Fs[i]}")
print("=============================")

delta_tau = []
for i in range(len(t_log)):
    q =q_log[i,:]
    dq = qd_log[i,:]
    thunder_ahand.set_q(q)
    thunder_ahand.set_dq(dq)
    thunder_ahand.set_dqr(dq)
    Y = thunder_ahand.get_reg_G()
    Y = Y[:,[0,1,2,3,  10,11,12,13,  20,21,22,23,  30,31,32,33 ]]   # Neglect inertia 

    # add friction damping and static
    #Y_d = -np.diag(qd_log[i])
    #Y_s = -np.diag(np.tanh(40*qd_log[i]))#np.diag(2/(1+np.exp(-40*qd_log[i])) - 1)
    #Y = np.hstack([Y, Y_d, Y_s])
    tau_est = np.array(Y @ hat_Pi).flatten()
    delta_tau.append(tau_log[i,:] - tau_est)
delta_tau = np.array(delta_tau)

print("> Making Plot")

# --- 1. Plot joint trajectories ---
plt.figure(figsize=(12,8))
for i in range(4):
    plt.subplot(2,2,i+1)
    plt.plot(t_log, q_log[:,i], 'r', label='q')# Plot joint limits
    #plt.plot(t_log, q_log[:,i] - e_log[:,i], 'b', linestyle='dotted', label='q_ref')# Plot joint desired trajectory
    #plt.hlines(JOINT_LIMITS[joint_indices[i], 0], t_log[0], t_log[-1], colors='k', linestyles='dotted', linewidth=1)
    #plt.hlines(JOINT_LIMITS[joint_indices[i], 1], t_log[0], t_log[-1], colors='k', linestyles='dotted', linewidth=1)
    plt.title(f'Joint {JOINT_NAMES[i]}')
    plt.grid(True)
    if i==0:
        plt.legend()
plt.tight_layout()
plt.show(block=False)
# --- 1. Plot joint trajectories ---
plt.figure(figsize=(12,8))
for i in range(4):
    plt.subplot(2,2,i+1)
    plt.plot(t_log, qd_log[:,i], 'r', label='qd')# Plot joint limits
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