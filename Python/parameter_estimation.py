import time
import os
import matplotlib.pyplot as plt
from scipy.linalg import solve_continuous_lyapunov, block_diag


import mujoco
import mujoco.viewer
import numpy as np
from tqdm import tqdm

import sys
sys.path.append("/home/leo/thunder_dynamics/ahand_finger_generatedFiles/build") # Where the .so file is located. 
sys.path.append("/home/leo/thunder_dynamics/franka_as_generatedFiles/build") # Where the .so file is located. 
# Note: This is not needed if the .so file is in the same directory as the python script
from thunder_ahand_finger_py import thunder_ahand_finger
from thunder_franka_as_py import thunder_franka_as

RENDER = False  # Set to False to run without rendering (faster)
MESH_DIR = './description/urdf/meshes/'
URDF_PATH = './description/urdf/allegro_hand_description_right_B.urdf'
XML_PATH = './description/urdf/allegro_hand_description_right_B.xml'
# Joint limits from URDF
JOINT_LIMITS = np.array([
    [-0.3, 0.3],    # Index 0
    [-0.01, 1.6],   # Index 1
    [-0.07, 1.86],  # Index 2
    [-0.02, 2.01],  # Index 3
    [-0.26, 0.26],  # Middle 0
    [-0.21, 1.79],  # Middle 1
    [-0.12, 1.86],  # Middle 2
    [-0.21, 1.85],  # Middle 3
    [-0.26, 0.29],  # Ring 0
    [-0.21, 1.79],  # Ring 1
    [-0.12, 1.86],  # Ring 2
    [-0.21, 1.85],  # Ring 3
    [0.00, 1.78],   # Thumb 0
    [-0.26, 1.65],  # Thumb 1
    [-0.05, 1.85],  # Thumb 2
    [-0.09, 1.80],  # Thumb 3
])


# --- Adaptive init ---
Kp = 30
Kv = 15
Ad = np.vstack((np.hstack((np.zeros((4,4)),  np.eye(4))),
                np.hstack((-Kp*np.eye(4),  -Kv*np.eye(4)))))
Bd = np.vstack((np.zeros((4,4)),
                np.eye(4)))
Q = np.eye(8)

P = solve_continuous_lyapunov(Ad.T, -Q)   # P | A'P + P A = -Q (solve_continuous_lyapunov: X | AX + XA' = Q)

R_inv = block_diag(block_diag(0.001, 0.0001*np.eye(3), 0.001*np.eye(6)),
                   block_diag(0.001, 0.0001*np.eye(3), 0.001*np.eye(6)),
                   block_diag(0.001, 0.0001*np.eye(3), 0.001*np.eye(6)),
                   block_diag(0.001, 0.0001*np.eye(3), 0.001*np.eye(6)))  # Inverse of R matrix in parameter update law

# --- Storage arrays ---
t_log = []
q_log = []
e_log = []
qd_log = []
tau_ctr_log = []
hat_Pi_log = []
feed_forward_log = []


if not os.path.exists(XML_PATH):
  print("Loading existing URDF file")
  m = mujoco.MjModel.from_xml_path(URDF_PATH)
  mujoco.mj_saveLastXML(XML_PATH, m)
  print("Exported URDF to XML file")
  exit(0)


m = mujoco.MjModel.from_xml_path(XML_PATH)
d = mujoco.MjData(m)

thunder_ahand = thunder_ahand_finger()
thunder_ahand.load_conf("/home/leo/thunder_dynamics/ahand_finger_generatedFiles/ahand_finger_conf.yaml")


q_bar = (JOINT_LIMITS[:,1] + JOINT_LIMITS[:,0])/2   # Mid position used to target configuration

q_t = np.zeros(16)
dq_t = np.zeros(16)
tau = np.zeros(16)
hat_Pi = thunder_ahand.get_par_DYN()

print("=== Dynamic parameters ======")
print("PI size:", hat_Pi.shape)
print("PI:", hat_Pi)
print("m: ", hat_Pi[[0, 10, 20, 30]])

print("CoM_0: ", hat_Pi[1:4])
print("Ib_0: ", hat_Pi[4:10])

print("CoM_1: ", hat_Pi[11:14])
print("Ib_1: ", hat_Pi[14:20])

print("CoM_2: ", hat_Pi[21:24])
print("Ib_2: ", hat_Pi[24:30])

print("CoM_3: ", hat_Pi[31:34])
print("Ib_3: ", hat_Pi[34:40])
print("=============================")

pbar = tqdm(total=60, desc="Simulation Time", unit="s", ncols=80)
if RENDER:
    # --- With viewer ---
    with mujoco.viewer.launch_passive(m, d) as viewer:
        start = time.time()
        while viewer.is_running() and time.time() - start < 60:
            step_start = time.time()
            t = step_start - start

            # --- Compute reference trajectories ---
            for i in range(16):
                amplitude = 0.5 * (q_bar[i] - JOINT_LIMITS[i,0])
                frequency = 0.1
                q_t[i] = amplitude * np.sin(2 * np.pi * frequency * t) + q_bar[i]
                dq_t[i] = 2 * np.pi * frequency * amplitude * np.cos(2 * np.pi * frequency * t)

            # --- Adaptive Computed Torque ---
            M_hat = thunder_ahand.get_M()
            Yr = thunder_ahand.get_reg_M() + thunder_ahand.get_reg_C() + thunder_ahand.get_reg_G()
            hat_Pi = thunder_ahand.get_par_DYN()

            e = q_t[4:8] - d.qpos[4:8]
            de = dq_t[4:8] - d.qvel[4:8]
            x = np.hstack((e, de))

            tau[4:8] = Yr @ hat_Pi + M_hat @ (Kp*e + Kv*de)
            tau_ff = Yr @ hat_Pi
            d.ctrl = tau

            dhat_Pi = R_inv @ Yr.T @ np.linalg.inv(M_hat.T) @ Bd.T @ P @ x

            # --- Step physics ---
            mujoco.mj_step(m, d)

            # --- Update AHAND state ---
            thunder_ahand.set_q(d.qpos[4:8])
            thunder_ahand.set_dq(d.qvel[4:8])
            thunder_ahand.set_dqr(d.qvel[4:8])
            thunder_ahand.set_dqr(d.qacc[4:8])
            hat_Pi += dhat_Pi * m.opt.timestep
            thunder_ahand.set_par_DYN(hat_Pi)

            # --- update progress ---
            pbar.n = min(round(t),60)
            pbar.refresh()

            # --- Viewer modifications ---
            with viewer.lock():
                viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)
            viewer.sync()

            # --- Time control ---
            time_until_next_step = m.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

            # --- Log data ---
            t_log.append(t)
            q_log.append(d.qpos[4:8].copy())
            e_log.append(d.qpos[4:8].copy() - q_t[4:8])
            qd_log.append(d.qvel[4:8].copy())
            tau_ctr_log.append(tau[4:8].copy())
            hat_Pi_log.append(hat_Pi.copy())
            feed_forward_log.append(tau_ff.copy())

else:
    # --- Headless simulation (no rendering) ---
    start = time.time()
    t = 0.0
    while t < 60:  # 60 seconds of simulation
        step_start = time.time()
        t = step_start - start

        # --- Compute reference trajectories ---
        for i in range(16):
            amplitude = 0.5 * (q_bar[i] - JOINT_LIMITS[i,0])
            frequency = 0.1
            q_t[i] = amplitude * np.sin(2 * np.pi * frequency * t) + q_bar[i]
            dq_t[i] = 2 * np.pi * frequency * amplitude * np.cos(2 * np.pi * frequency * t)

        # --- Adaptive Computed Torque ---
        M_hat = thunder_ahand.get_M()
        Yr = thunder_ahand.get_reg_M() + thunder_ahand.get_reg_C() + thunder_ahand.get_reg_G()
        hat_Pi = thunder_ahand.get_par_DYN()

        e = q_t[4:8] - d.qpos[4:8]
        de = dq_t[4:8] - d.qvel[4:8]
        x = np.hstack((e, de))

        tau[4:8] = Yr @ hat_Pi + M_hat @ (Kp*e + Kv*de)
        tau_ff = Yr @ hat_Pi
        d.ctrl = tau

        dhat_Pi = R_inv @ Yr.T @ np.linalg.inv(M_hat.T) @ Bd.T @ P @ x

        # --- Step physics ---
        mujoco.mj_step(m, d)

        # --- Update AHAND state ---
        thunder_ahand.set_q(d.qpos[4:8])
        thunder_ahand.set_dq(d.qvel[4:8])
        thunder_ahand.set_dqr(d.qvel[4:8])
        hat_Pi += dhat_Pi * m.opt.timestep
        thunder_ahand.set_par_DYN(hat_Pi)

        # --- Update progress bar ---
        pbar.n = min(round(t),60)
        pbar.refresh()

        # --- Time control ---
        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

        # --- Log data ---
        t_log.append(t)
        q_log.append(d.qpos[4:8].copy())
        e_log.append(d.qpos[4:8].copy() - q_t[4:8])
        qd_log.append(d.qvel[4:8].copy())
        tau_ctr_log.append(tau[4:8].copy())
        hat_Pi_log.append(hat_Pi.copy())
        feed_forward_log.append(tau_ff.copy())


print("Making Plot")
# --- Convert logs to numpy arrays ---
t_log = np.array(t_log)
q_log = np.array(q_log)
e_log = np.array(e_log)
qd_log = np.array(qd_log)
tau_ctr_log = np.array(tau_ctr_log)
hat_Pi_log = np.array(hat_Pi_log)
feed_forward_log = np.array(feed_forward_log)

joint_indices = [4, 5, 6, 7]


# --- 1. Plot joint trajectories ---
plt.figure(figsize=(12,8))
for i in range(4):
    plt.subplot(2,2,i+1)
    plt.plot(t_log, q_log[:,i], 'b', label='q')# Plot joint limits
    plt.plot(t_log, q_log[:,i] - e_log[:,i], 'b', linestyle='dotted', label='q_ref')# Plot joint desired trajectory
    plt.hlines(JOINT_LIMITS[joint_indices[i], 0], t_log[0], t_log[-1], colors='k', linestyles='dotted', linewidth=1)
    plt.hlines(JOINT_LIMITS[joint_indices[i], 1], t_log[0], t_log[-1], colors='k', linestyles='dotted', linewidth=1)
    plt.title(f'Joint {joint_indices[i]}')
    plt.grid(True)
    if i==0:
        plt.legend()
plt.tight_layout()
plt.show(block=False)
# --- 2. Plot joint errors ---
plt.figure(figsize=(12,8))
for i in range(4):
    plt.subplot(2,2,i+1)
    plt.plot(t_log, e_log[:,i], 'b', label='e_q')# Plot joint limits
    plt.title(f'Error Joint {joint_indices[i]}')
    plt.grid(True)
    if i==0:
        plt.legend()
plt.tight_layout()
plt.show(block=False)


# --- 3. Plot torques ---
plt.figure(figsize=(12,8))
for i in range(4):
    plt.subplot(2,2,i+1)
    plt.plot(t_log, tau_ctr_log[:,i], label = 'tau')
    plt.xlabel('Time [s]')
    plt.ylabel('Torque [Nm]')
    plt.title(f'Joint {joint_indices[i]}')
    plt.grid(True)
    if i==0:
        plt.legend()
plt.tight_layout()
plt.show(block=False)

# --- 3.b Plot torques ---
plt.figure(figsize=(12,8))
for i in range(4):
    plt.subplot(2,2,i+1)
    plt.plot(t_log, feed_forward_log[:,i], label = 'tau_ff')
    plt.xlabel('Time [s]')
    plt.ylabel('Feed-Forward Torque [Nm]')
    plt.title(f'Joint {joint_indices[i]}')
    plt.grid(True)
    if i==0:
        plt.legend()
plt.tight_layout()
plt.show(block=False)

# --- 4. Plot identified parameters over time ---
plt.figure(figsize=(14,10))

# --- 4.a Masses ---
plt.subplot(3, 2, 1)
for i, idx in enumerate([0, 10, 20, 30]):
    plt.plot(t_log, hat_Pi_log[:, idx], label=f"m_{i}")
plt.title("Mass Estimates")
plt.xlabel("Time [s]")
plt.ylabel("Mass [kg]")
plt.grid(True)
plt.legend()

# --- 4.b Centers of Mass ---
plt.subplot(3, 2, 2)
for i, start in enumerate([1, 11, 21, 31]):
    plt.plot(t_log, hat_Pi_log[:, start],   label=f"CoM_{i}_x")
    plt.plot(t_log, hat_Pi_log[:, start+1], label=f"CoM_{i}_y")
    plt.plot(t_log, hat_Pi_log[:, start+2], label=f"CoM_{i}_z")
plt.title("Center of Mass Estimates")
plt.xlabel("Time [s]")
plt.ylabel("CoM [m]")
plt.grid(True)
plt.legend(ncol=2)

# --- 4.c Inertia tensors ---
inertia_ranges = [(4,10), (14,20), (24,30), (34,40)]
for i, (start, end) in enumerate(inertia_ranges):
    plt.subplot(3, 2, i+3)
    for j in range(start, end):
        plt.plot(t_log, hat_Pi_log[:, j], label=f"Ib_{i}_{j-start}")
    plt.title(f"Inertia Parameters (Body {i})")
    plt.xlabel("Time [s]")
    plt.ylabel("Inertia [kg·m²]")
    ymax = np.min(hat_Pi_log[:, j])
    ymin = np.min(hat_Pi_log[:, j])
    plt.ylim(ymin - 0.1*ymin, ymax + 0.1*ymax)
    plt.grid(True)
    if i == 0:
        plt.legend(ncol=2)

plt.tight_layout()
plt.show()

#exit(0)