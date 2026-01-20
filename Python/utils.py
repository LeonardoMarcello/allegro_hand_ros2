import time
import os
import sys
import shutil
import matplotlib.pyplot as plt
from scipy.linalg import solve_continuous_lyapunov, block_diag
import rosbag2_py
from rclpy.serialization import serialize_message, deserialize_message
from sensor_msgs.msg import JointState
import casadi

import numpy as np
from tqdm import tqdm

import pandas as pd

from scipy.signal import butter, filtfilt, savgol_filter

# =========================================================================================================================== 
# IMPORT THUNDER FILES
rel_paths = ["./ahand_finger_generatedFiles/build", "./ahand_thumb_generatedFiles/build", "./franka_generatedFiles/build"]
for rel_path in rel_paths:
    dir_path = os.path.dirname(os.path.abspath(__file__))
    build_path = os.path.join(dir_path, rel_path)
    if os.path.exists(build_path):
        sys.path.append(os.path.abspath(build_path))
    else:
        print(f"Warning: path '{build_path}' does not exist.")

from thunder_ahand_finger_py import thunder_ahand_finger
from thunder_ahand_thumb_py import thunder_ahand_thumb
from thunder_franka_py import thunder_franka

# MESH_DIR = './description/urdf/meshes/'
# URDF_PATH = './description/urdf/allegro_hand_description_right_B.urdf'
# XML_PATH = './description/urdf/allegro_hand_description_right_B.xml'
# BAG_PATH = '/home/leo/Desktop/allegro_hand_ros2_ws/workdir/allegro_hand_pb_gc_bag'
# JOINT_NAMES = ['joint_4_0', 'joint_5_0', 'joint_6_0', 'joint_7_0']
# SAVE_FILENAME = '/home/leo/Desktop/ahand_dynamic_gravity_compensation.txt'

# MASSES_URDF = [0.0176, 0.086, 0.0367, 0.0057] # [Kg]
# HCK_MASSES = [0.0168, 0.1023, 0.0405, 0.0603] # [Kg]
# HCK_KP = [1, 1, 1, 1] # [Nm/rad]
# HCK_KD = [0.04, 0.15, 0.04, 0.04] # [Nm/(rad/s^2)]
# HCK_D = [0.015*0.004*np.sqrt(80), 0.015*0.008*np.sqrt(90), 0.015*0.008*np.sqrt(90), 0.015*0.004*np.sqrt(80)] # [Nm/(rad/s^2)]

# =========================================================================================================================== 
# READER

# Bag Read/Write
def read_bag(file_path, topic_name = '/allegroHand_0/joint_states', joint_names=None, store=False, path_out=None):
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
        if topic == topic_name:
        #if topic == '/allegroHand_0/torque_cmd':
            msg = deserialize_message(data, JointState)
            msg_joint_names = msg.name

            # select specific joints
            indices = [msg_joint_names.index(name) for name in joint_names if name in msg_joint_names]
            selected_joint_names = [msg_joint_names[i] for i in indices]
            if first:
                msg_joint_names = msg.name
                # get header
                pos_header    = [f"pos_{name}" for name in selected_joint_names]
                effort_header = [f"eff_{name}" for name in selected_joint_names]
                vel_header    = [f"vel_{name}" for name in selected_joint_names]
                #acc_header    = [f"acc_{name}" for name in selected_joint_names]
                #header = ['t'] + pos_header + vel_header + acc_header + effort_header
                header = ['t'] + pos_header + effort_header + vel_header
                first = False
            elif msg_joint_names != msg.name:
                raise ValueError("Joint names do not match")

            # Store data
            selected_positions = [msg.position[i] for i in indices]
            selected_velocities = [msg.velocity[i] for i in indices]
            selected_effort = [msg.effort[i] for i in indices]
            row = [t/1e9] + selected_positions + selected_effort + selected_velocities
            dataset.append(row)

    # store in dataframe
    df = pd.DataFrame(dataset, columns=header)
    if store:
        df.to_csv(file_path, index=False)

    return df
def read_csv(file_path, joint_names = None, store=False, path_out=None):
    # Read the CSV
    df_raw = pd.read_csv(file_path)

    # Infer joint names if not provided
    if joint_names is None:
        joint_names = sorted(set(
            c.replace('_pos', '').replace('_vel', '').replace('_effort', '')
            for c in df_raw.columns if any(x in c for x in ['_pos', '_vel', '_effort'])
        ))

    # Input column names (from CSV)
    pos_in = [f"{j}_pos" for j in joint_names]
    eff_in = [f"{j}_effort" for j in joint_names]
    vel_in = [f"{j}_vel" for j in joint_names]

    # Output column names (desired format)
    pos_out = [f"pos_{j}" for j in joint_names]
    eff_out = [f"eff_{j}" for j in joint_names]
    vel_out = [f"vel_{j}" for j in joint_names]

    # Build final DataFrame with renamed columns
    df = pd.DataFrame()
    df['t'] = range(len(df_raw))  # add time index
    for in_cols, out_cols in zip([pos_in, eff_in, vel_in], [pos_out, eff_out, vel_out]):
        for ci, co in zip(in_cols, out_cols):
            if ci in df_raw.columns:
                df[co] = df_raw[ci]
            else:
                df[co] = None  # if missing, fill with NaN

    # Optionally store
    if store:
        out_path = path_out or file_path.replace('.csv', '_parsed.csv')
        df.to_csv(out_path, index=False)

    return df

def write_filtered_bag(file_path, t_log, q_log, tau_log, joint_names_filtered):
    if os.path.exists(file_path):
        print(f"Overwriting existing bag: {file_path}")
        shutil.rmtree(file_path)
    all_joint_names = [f'joint_{i}_0' for i in range(15)]
    # Create writer
    writer = rosbag2_py.SequentialWriter()
    storage_options = rosbag2_py.StorageOptions(uri=file_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    writer.open(storage_options, converter_options)

    # Create topic metadata
    topic_name = '/allegroHand_0/torque_cmd'
    topic_type = 'sensor_msgs/msg/JointState'
    writer.create_topic(rosbag2_py.TopicMetadata(name=topic_name,
                                                 type=topic_type,
                                                 serialization_format='cdr'))

    # Map filtered indices to full joint list
    idx_map = [all_joint_names.index(j) if j in all_joint_names else None for j in joint_names_filtered]

    # Write data
    for i in range(len(t_log)):
        msg = JointState()
        msg.header.stamp.sec = int(t_log[i])
        msg.header.stamp.nanosec = int((t_log[i] - int(t_log[i])) * 1e9)
        msg.name = all_joint_names
        # Initialize all zeros
        q_full = np.zeros(15)
        tau_full = np.zeros(15)
        # Fill available joints
        for j, idx in enumerate(idx_map):
            if idx is not None:
                q_full[idx] = q_log[i, j]
                tau_full[idx] = tau_log[i, j]

        msg.position = q_full.tolist()
        msg.effort = tau_full.tolist()
        writer.write(topic_name, serialize_message(msg), int(t_log[i]*1e9))

    print(f"Filtered bag written to {file_path} with all joints 0-14")

# ============================================
# PARAMETER REDUCTION
# ============================================
def Lambda(alpha, a, d, theta):
    sa = np.sin(alpha)
    ca = np.cos(alpha)
    st = np.sin(theta)
    ct = np.cos(theta)

    ssa = np.sin(alpha)*np.sin(alpha)
    cca = np.cos(alpha)*np.cos(alpha)
    sst = np.sin(theta)*np.sin(theta)
    cct = np.cos(theta)*np.cos(theta)

    csa = np.cos(alpha)*np.sin(alpha)
    cst = np.cos(theta)*np.sin(theta)

    L = np.zeros((10,10))

    # Mass
    L[:,0] = np.array([1,   a,-d*sa,d*ca,   d**2, a*d*sa, -a*d*ca, a**2 + d**2*cca, d**2*csa, a**2 + d**2*ssa])
    # m*COM
    L[:,1] = np.array([0,   ct, st*ca, st*sa,   0, -a*st*ca + d*ct*sa, -a*st*sa - d*ct*ca, 2*(a*ct + d*st*csa), d*st*(ssa-cca), 2*(a*ct - d*st*csa)])
    L[:,1] = np.array([0,   -st, ct*ca, ct*sa,  0, -a*ct*ca + d*st*sa, -a*ct*sa - d*st*ca, 2*(-a*st + d*ct*csa), d*ct*(ssa-cca), 2*(-a*st - d*ct*csa)])
    L[:,3] = np.array([0,   0, -sa, ca,         2*d, a*sa, -a*ca, 2*d*cca, 2*d*csa, 2*d*ssa])
    # Inertia
    L[:,4] = np.array([0,   0, 0, 0,  cct, cst*ca, cst*sa, sst*cca, sst*csa, sst*ssa])
    L[:,5] = np.array([0,   0, 0, 0,  -2*cst, (cct -sst)*ca, (cct -sst)*sa, 2*cst*cca, 2*cst*csa, 2*cst*ssa])
    L[:,6] = np.array([0,   0, 0, 0,  0, -ct*sa, -ct*sa, -2*st*csa, st*(cca-ssa), 2*st*csa])
    L[:,7] = np.array([0,   0, 0, 0,  sst, -cst*ca, -cst*sa, cct*cca, cct*csa, cct*ssa])
    L[:,8] = np.array([0,   0, 0, 0,  0, st*sa, -st*sa, -2*ct*csa, ct*(cca-ssa), 2*ct*csa])
    L[:,9] = np.array([0,   0, 0, 0,  0, 0, 0, sst, -csa, cca])

    return L
def S(k, n, add_rotor_inertia = False):
    # selettore parametri k s.t. pi_r_k = S_k * pi_r
    S = np.zeros((10, n*10))
    S[:, k*10:(k+1)*10] = np.eye(10)

    if add_rotor_inertia:
        S_I = np.zeros((1, n))
        S_I[:, k] = 1
        S = block_diag(S, S_I)

    return S
def regrouping(DH, robot, add_rotor_inertia = False):
    start = time.time()
    n = len(DH['joints'])
    pi_r = np.ones((n*10,1))                                    # original regressor parameter vector
    pi_I = np.ones((n,1))                                       # rotor ineria params vector (may be unused)
    if add_rotor_inertia: pi = np.vstack([pi_r, pi_I])
    else: pi = pi_r

    hat_pi_r =[]                                                # reduced parameter vector
    g = np.array([0, 0, -9.81])                                 # gravity vector (<- nice to have from thunder_ahand)

    E = []
    H = []
    W = []

    tolerance = 1e-6                                           # use for zero detection

    # 1_ get r1, r2 and p1, rp1 ------------------
    # a) r1 = first revolute joint
    for i in range(n):
        if DH['joints'][i] == 'R':
            r1 = i
            print("r1 = ", r1)
            break
        if i==n-1:
            print("revolute joint not found.")
            r1 = n
            break
    T_0_r1 = getattr(robot, f"get_T_0_{r1+1}")()    # _Oss_. T_0_0 is the world frame not Frame 0
    k_r1 = T_0_r1[0:3,2]                                    # z axis of frame r1 in world frame

    # b) r2 = Next revolute joint not aligned to r1
    for i in range(r1+1,n):
        if DH['joints'][i] == 'R':
            T_0_i = getattr(robot, f"get_T_0_{i+1}")()
            k_i = T_0_i[0:3,2]
            if np.abs(k_r1.transpose() @ k_i) < 1 - 1e-14:  # if z_i axis is not aligned to z_r1 axis
                r2 = i
                print("r2 = ", r2)
                break
            if i==n-1:
                r2 = 0
                print("r2 not found")
    T_0_r2 = getattr(robot, f"get_T_0_{r2+1}")()    # Oss. T_0_0 is the world frame not Frame 0
    k_r2 = T_0_r2[0:3,2]  # z axis of frame r1 in world frame


    # c) p1 = first prismatic joint
    for i in range(n):
        if DH['joints'][i] == 'P':
            p1 = i
            print("p1 = ", p1)
            break
        if i==n-1:
            print("prismatic joint not found.")
            p1 = -1
            break

    # d) rp1 = Last revolute joint preceding p1
    for i in range(p1-1, -1, -1):
        if DH['joints'][i] == 'R':
            rp1 = i
            print("rp1 = ", rp1)
            break
        elif i==0:
            rp1 = n
            print("rp1 not found")


    # 2_ Select and reduce ------------------------------------
    for i in range(n-1, -1, -1):
        if DH['joints'][i] == 'R':
        #  ----------- REVOLUTE ----------------------------
            # 1) Get i-th param
            S_i = S(i, n, add_rotor_inertia)    # Link param selector
            pi_i = S_i @ pi                     # pi_r_i = [m, cx,cy,cz, I_xx,I_xy,I_xz,I_yy,I_yz,I_zz(, I_r)]'

            # 2) Get type of param reduction (compute E_i)
            E_i = block_diag(1,
                               np.eye(3),
                                        np.array([[1,0,0,-1,0,0],         # regroup I_yy in I_xx
                                                  [0,1,0, 0,0,0],
                                                  [0,0,1, 0,0,0],
                                                  [0,0,0, 1,0,0],
                                                  [0,0,0, 0,1,0],
                                                  [0,0,0, 0,0,1]]))
            print(f"Deleting I_yy, mc_z, m of link {i}...")
            E_i = E_i[[ 1,2, 4,5,6,8,9],:]              # Remove m, m_cz, I_yy
            if add_rotor_inertia: 
                E_i =  block_diag(E_i, 1)               # (add rotor inertia block if needed)

            if add_rotor_inertia and i == r2 and np.abs(k_r2 @ k_r1) < tolerance:
                print(f"Deleting I_r of link {i}...")
                # r2 joint axis orthogonal to r1 joint axis
                E_i[6,10] = 1       # regroup I_r in I_zz
                E_i = E_i[0:7,:]          # Remove I_r
            if add_rotor_inertia and i == r1:
                print(f"Deleting I_r of link {i}...")
                # joint r1
                E_i[6,10] = 1       # regroup I_r in I_zz
                E_i = E_i[0:7,:]          # Remove I_r

            if r1<=i and i<r2:
                print(f"Deleting I_xx, I_xy, I_xz, I_yz of link {i}...")
                E_i = np.vstack([E_i[[ 0,1],:], E_i[6:,:]])                     # remove I_xx, I_xy, I_xz, I_yz


                T_0_i = getattr(robot, f"get_T_0_{i+1}")()
                k_i = T_0_i[0:3,2]      # z axis of frame i in world frame
                k_r1 = T_0_r1[0:3,2]    # z axis of frame r1 in world frame
                k_g = g/np.linalg.norm(g)

                if np.abs(k_i.transpose() @ k_r1) > 1 - tolerance:  # if z_i axis aligned to z_r1 axis
                    if i == 0:
                        if np.abs(k_i.transpose() @ k_g) > 1 - tolerance:   # if z_i axis aligned to gravity
                            print(f"Deleting mc_x, mc_y of link {i}...")
                            E_i = E_i[2:,:]                                               # remove mc_x, mc_y
                        #print(f"Deleting mc_x, mc_y of link {i}...")
                        #E_i = E_i[2:,:]                                               # (ori always delete) remove mc_x, mc_y
                    else:
                        for j in range(i):
                            k_j = getattr(robot, f"get_T_0_{j+1}")()[0:3,2]
                            if not(np.abs(k_j.transpose() @ k_r1) > 1 - tolerance and np.abs(k_j.transpose() @ k_g) > 1 - tolerance): # if NOT z_j axis aligned to z_r1 axis and to gravity
                                break
                            if j==i-1:
                                print(f"Deleting mc_x, mc_y of link {i}...")
                                E_i = E_i[2:,:]                                               # remove mc_x, mc_y

            hat_pi_i = E_i @ pi_i.reshape(-1, 1)  # Extracting reduced params from i-th link

            # 3) Remapping on previous link (compute H_iminus)
            if i>0:
                iminus_L_i = Lambda(DH['alpha'][i], DH['a'][i], DH['d'][i], DH['theta'][i])
                if not(add_rotor_inertia):
                    # -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
                    #                           m,                               cx,cy,        cz,                             I_xx,I_xy,I_xz,               I_yy,                                I_yz,I_zz,     |     pi_r
                    # -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
                    H_iminus = np.hstack([iminus_L_i[:,0].reshape(-1, 1),  np.zeros((10,2)),iminus_L_i[:,3].reshape(-1, 1),  np.zeros((10,3)),(iminus_L_i[:,4] + iminus_L_i[:,7]).reshape(-1, 1),np.zeros((10,2)), np.eye(10)])
                    pi[10*(i-1):10*i,:] = H_iminus @ np.vstack([pi_i,
                                                                S(i-1,n)@pi])
                else:
                    # -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
                    #                                                m,                      cx,cy,        cz,                          I_xx,I_xy,I_xz,               I_yy,                                  I_yz,I_zz,Ir   |      pi_r, pi_I
                    # -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
                    H_iminus = np.vstack([np.hstack([iminus_L_i[:,0].reshape(-1, 1),  np.zeros((10,2)),iminus_L_i[:,3].reshape(-1, 1),  np.zeros((10,3)),(iminus_L_i[:,4] + iminus_L_i[:,7]).reshape(-1, 1),np.zeros((10,3)), np.eye(10), np.zeros((10,1))]),
                                          np.array([             0,                           0,0,         0,                              0,0,0,                       0,                                       0,0,0,        0,0,0,0,0,0,0,0,0,0,1])])

                    tmp = H_iminus @ np.vstack([pi_i,
                                                S(i-1,n, add_rotor_inertia) @ pi])
                    pi[10*(i-1):10*i,:] = tmp[:10,:]
                    pi[10*n + i,:] = tmp[10,:]

            # 4) Get relationship parameters-reduced parameter i-th (Compute of W_i)
            if i==n-1:
                # last link
                W_i = S_i
                H_i = np.eye(10)
                if (add_rotor_inertia): H_i = block_diag(H_i, 1)
                H = [H_i] + H
            else:
                H_i = H[0]
                H_iplus = H[1]
                W_iplus = W[0]
                W_i = np.vstack([H_iplus @ W_iplus,
                                 S_i])
            if i>0:  H = [H_iminus] + H
            W = [W_i] + W
            E = [E_i] + E


            hat_pi_r = hat_pi_i.flatten().tolist() + hat_pi_r

        #  ----------- PRISMATIC ----------------------------
        elif DH['joints'][i] == 'P':
            # 1) Get i-th param
            S_i = S(i, n, add_rotor_inertia)                   # param selector
            pi_i = S_i @ pi                                    # pi_r_i = [m, cx,cy,cz, I_xx,I_xy,I_xz,I_yy,I_yz,I_zz(, Ir)]

            # 2) Get type of param reduction (compute E_i)
            E_i = np.eye(10)
            print(f"Deleting I_xx, I_xy, I_xz, I_yy, I_yz, I_zz of link {i}...")
            E_i = E_i[0:4,:]

            if r1 < i and  i < r2:
                T_0_i = getattr(robot, f"get_T_0_{i+1}")()
                k_i = T_0_i[0:3,2]      # z axis of frame i in world frame
                k_r1 = T_0_r1[0:3,2]    # z axis of frame r1 in world frame
                k_g = g/np.linalg.norm(g)

                if np.abs(k_i.transpose() @ k_r1) > 1 - tolerance:       # if z_i axis parallel to z_r1 axis
                    print(f"Deleting mc_z of link {i}...")
                    E_i = E_i[0:3,:]                        # remove mc_z

                else:                                                    # if z_i axis is not parallel to z_r1 axis
                    T_i_r1 = np.linalg.inv(T_0_i) @ T_0_r1
                    i_k_r1 = T_i_r1[0:3,2]
                    if i_k_r1[3] > tolerance:
                        E_i[1,3] = -i_k_r1[1] / i_k_r1[3]     # Regroup mc_z in mc_x
                        E_i[2,3] = -i_k_r1[2] / i_k_r1[3]     # Regroup mc_z in mc_x
                        E_i = E_i[0:3,:]                        # remove mc_z
                    elif i_k_r1[0] > tolerance and i_k_r1[1] > tolerance:
                        E_i[1,3] = -i_k_r1[1] / i_k_r1[2]     # Regroup mc_y in mc_x
                        E_i = E_i[[0,1,3],:]                    # remove mc_y
                    elif i_k_r1[2] > tolerance :
                        E_i = E_i[[0,1,3],:]                    # remove mc_y
                    else:
                        E_i = E_i[[0,2,3],:]                    # remove mc_x
            elif i < r1:
                print(f"Deleting mc_x, mc_y, mc_z of link {i}...")
                E_i = E_i[0,0]                                                      # remove mc_x, mc_y, mc_z


            if add_rotor_inertia: E_i =  block_diag(E_i, 1)               # (add rotor inertia block if needed)
            if add_rotor_inertia and i == p1:
                T_0_p1 = getattr(robot, f"get_T_0_{p1+1}")()    # Oss. T_0_0 is the world frame not Frame 0
                k_p1 = T_0_p1[0:3,2]  # z axis of frame r1 in world frame
                T_0_rp1 = getattr(robot, f"get_T_0_{rp1+1}")()    # Oss. T_0_0 is the world frame not Frame 0
                k_rp1 = T_0_rp1[0:3,2]  # z axis of frame r1 in world frame
                k_g = g/np.linalg.norm(g)
                if (np.abs(k_p1.transpose() @ k_g) < tolerance) and (p1 == 1 or  np.abs(k_p1.transpose() @ k_rp1) > 1 - tolerance):
                    print(f"Deleting I_r of link {i}...")
                    # r2 joint axis orthogonal to r1 joint axis
                    E_i[0,10] = 1       # regroup I_r in m
                    E_i[:-1,:]          # Remove I_r

            hat_pi_i = E_i @ pi_i.reshape(-1, 1)  # Extracting reduced params from i-th link

            # 3) Remapping on previous link (compute H_iminus)
            if i>0:
                iminus_L_i = Lambda(DH['alpha'][i], DH['a'][i], DH['d'][i], DH['theta'][i])
                if not(add_rotor_inertia):
                    # ------------------------------------------------------------------------------------------
                    #                      m, cx,cy,cz,      I_xx,I_xy,I_xz,I_yy,I_yz,I_zz,     |     pi_r
                    # ------------------------------------------------------------------------------------------
                    H_iminus = np.hstack([np.zeros((10,4)),     iminus_L_i[:,4:10],                 np.eye(10)])
                    pi[10*(i-1):10*i,:] = H_iminus @ np.vstack([pi_i,
                                                                S(i-1,n)@pi])
                else:
                    # -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
                    #                               m,cx,cy,cz,         I_xx,I_xy,I_xz,I_yy,I_yz,I_zz,           Ir          |      pi_r, pi_I
                    # -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
                    H_iminus = np.vstack([np.hstack([np.zeros((10,4)),     iminus_L_i[:,4:10],             np.zeros((10,1)),    np.eye(10), np.zeros((10,1))]),
                                          np.array( [ 0,0,0,0,             0,0,0,0,0,0,                           0,            0,0,0,0,0,0,0,0,0,0,1])])

                    tmp = H_iminus @ np.vstack([pi_i,
                                                S(i-1,n, add_rotor_inertia) @ pi])
                    pi[10*(i-1):10*i,:] = tmp[:10,:]
                    pi[10*n + i,:] = tmp[10,:]


            # 4) Get relationship parameters-reduced parameter i-th (Compute of W_i)
            if i==n-1:
                # last link
                W_i = S_i
                H_i = np.eye(10)
                if (add_rotor_inertia): H_i = block_diag(H_i, 1)
                H = [H_i] + H
            else:
                H_i = H[0]
                H_iplus = H[1]
                W_iplus = W[0]
                W_i = np.vstack([H_iplus @ W_iplus,
                                 S_i])
            if i>0:  H = [H_iminus] + H
            W = [W_i] + W
            E = [E_i] + E


            hat_pi_r = hat_pi_i.flatten().tolist() + hat_pi_r

    beta = []
    for i in range(n):
        beta.append(E[i] @ H[i] @ W[i])
    beta = np.vstack(beta)

    # ----- Final reporting
    end = time.time()
    if add_rotor_inertia: print(f"Regrouping completed in {1000*(end - start)} ms. Parameters reduced from {len(pi_r.tolist())+len(pi_I.tolist())} to ", len(hat_pi_r))
    else: print(f"Regrouping completed in {1000*(end - start)} ms. Parameters reduced from {len(pi_r.tolist())} to ", len(hat_pi_r))



    return hat_pi_r, beta

def save_beta(filename, beta, hat_Pi):
    if not os.path.exists(filename):
        beta_pinv = np.linalg.pinv(beta)
        print("Creating sparse beta and pi_hat C++ header file...")
        with open(filename, "w") as f:
            f.write("// Auto-generated sparse beta and pi_hat\n")#include "beta_sparse.h"
            f.write("/* Usage\n")
            f.write(f"#include '{filename}'\n\n")
            f.write("int main() {\n")
            f.write("    Eigen::SparseMatrix<double> beta;\n")
            f.write("    load_beta(beta);\n")
            f.write("\n")
            f.write("    auto pi_hat = load_pi_hat();\n")
            f.write("\n")
            f.write("    // Example usage:\n")
            f.write("    Eigen::VectorXd tau = Y * beta.pinv() * pi_hat;\n")
            f.write("\n")
            f.write('    std::cout << "tau = \n" << tau << std::endl;\n')
            f.write("}\n")
            f.write("*/\n\n")

            f.write("#include <Eigen/Dense>\n")
            f.write("#include <Eigen/Sparse>\n")
            f.write("#include <vector>\n\n")
            f.write("{\n\n")

            # 1) Beta as sparse matrix---
            N, M = beta.shape
            # Triplets
            f.write("inline void load_beta(Eigen::SparseMatrix<double>& beta) {\n")
            f.write(f"    beta.resize({N}, {M});\n")
            f.write("    std::vector<Eigen::Triplet<double>> triplets;\n")
            f.write(f"    triplets.reserve({np.count_nonzero(beta)});\n\n")

            rows, cols = np.nonzero(beta)
            for r, c in zip(rows, cols):
                f.write(f"    triplets.emplace_back({r}, {c}, {beta[r,c]:.16g});\n")

            f.write("\n    beta.setFromTriplets(triplets.begin(), triplets.end());\n")
            f.write("}\n\n")

            # 2) Beta Pseudo-Inverse as sparse matrix---
            N, M = beta_pinv.shape
            # Triplets
            f.write("inline void load_beta_pinv(Eigen::SparseMatrix<double>& beta_pinv) {\n")
            f.write(f"    beta_pinv.resize({N}, {M});\n")
            f.write( "    std::vector<Eigen::Triplet<double>> triplets;\n")
            f.write(f"    triplets.reserve({np.count_nonzero(beta_pinv)});\n\n")

            rows, cols = np.nonzero(beta_pinv)
            for r, c in zip(rows, cols):
                f.write(f"    triplets.emplace_back({r}, {c}, {beta_pinv[r,c]:.16g});\n")

            f.write("\n    beta_pinv.setFromTriplets(triplets.begin(), triplets.end());\n")
            f.write("}\n\n")

            # 3) pi_hat as dense vector----
            f.write(f"inline Eigen::Matrix<double, {hat_Pi.shape[0]}, 1> load_pi_hat() {{\n")
            f.write(f"    Eigen::Matrix<double, {hat_Pi.shape[0]}, 1> v;\n")
            f.write("    v << \n")
            for i in range(hat_Pi.shape[0]):
                sep = "," if i < hat_Pi.shape[0]-1 else ";"
                f.write(f"        {hat_Pi[i,0]:.16g}{sep}\n")
            f.write("    return v;\n}\n")
        print(f"{filename} created successfully.")
    else:
        print(f"{filename} already exists, skipping creation.")