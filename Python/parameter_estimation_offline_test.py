import time
import os
import matplotlib.pyplot as plt
from scipy.linalg import solve_continuous_lyapunov, block_diag

import casadi

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

TIME = 60  # Seconds of simulation [s]
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


# --- PD gains  init ---
Kp = 70*np.diag([1.5,2.0,1.8,1.3])
Kd = np.diag([0.1,0.11,0.12,0.13])
Ki = 20*np.diag([1,1,1,1])

# --- Storage arrays ---
t_log = []
q_log = []
qd_log = []
qdd_log = []
e_log = []
tau_ctr_log = []
Y_log = []


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

pbar = tqdm(total=TIME, desc="Simulation Time", unit="s", ncols=80)
# --- With viewer ---
with mujoco.viewer.launch_passive(m, d) as viewer:
    start = time.time()
    while viewer.is_running() and time.time() - start < TIME:
        step_start = time.time()
        t = step_start - start

        # --- Compute reference trajectories ---
        for i in range(16):
            amplitude = 0.5 * (q_bar[i] - JOINT_LIMITS[i,0])
            frequency = 0.1
            q_t[i] = amplitude * np.sin(2 * np.pi * frequency * t) + q_bar[i]
            dq_t[i] = 2 * np.pi * frequency * amplitude * np.cos(2 * np.pi * frequency * t)

        e = q_t[4:8] - d.qpos[4:8]
        de = dq_t[4:8] - d.qvel[4:8]

        # PID
        tau[4:8] = Kp@e + Kd@de        # <-- I-T-M-R
        d.ctrl = tau

        # --- Step physics ---
        mujoco.mj_step(m, d)

        # --- Update AHAND state ---
        thunder_ahand.set_q(d.qpos[4:8])
        thunder_ahand.set_dq(d.qvel[4:8])
        thunder_ahand.set_dqr(d.qvel[4:8])
        thunder_ahand.set_ddqr(d.qacc[4:8])
        Y = thunder_ahand.get_reg_M() + thunder_ahand.get_reg_C() + thunder_ahand.get_reg_G()

        # --- update progress ---
        pbar.n = min(round(t),TIME)
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
        qd_log.append(d.qvel[4:8].copy())
        qdd_log.append(d.qacc[4:8].copy())
        e_log.append(d.qpos[4:8].copy() - q_t[4:8])
        tau_ctr_log.append(tau[4:8].copy())
        Y_log.append(Y.copy())


pbar.close()

# --- Convert logs to numpy arrays ---
t_log = np.array(t_log)
q_log = np.array(q_log)
e_log = np.array(e_log)
qd_log = np.array(qd_log)
qdd_log = np.array(qdd_log)
tau_ctr_log = np.array(tau_ctr_log)


print("> Computing Dynamics param")
print("Number of samples (M): ", len(t_log))
# YY @ Pi = TTau, where YY (is Mnxp), TTau (is Mnx1) and Pi (is px1)
YY = np.array(Y_log[0])
TTau = np.array(tau_ctr_log[0,:]).reshape(4,1)
for i in range(1,len(t_log)):
    YY = np.vstack((YY,
                    np.array(Y_log[i])))
    TTau = np.vstack((TTau,
                      np.array(tau_ctr_log[i]).reshape(4,1)))
M = len(t_log)
n = thunder_ahand.get_numJoints()
p = Y_log[0].shape[1]
print("Shape YY (Mnxp): ", YY.shape)
print("Shape TTau (Mnx1): ", TTau.shape)

# Method 1 - Pinv
hat_Pi = np.linalg.pinv(YY)@TTau
print("Computed initial guess by pseudoinverse")
# Method 2 - Least Squares
hat_Pi_sym = casadi.SX.sym('Pi', p, 1)    # Parameters to estimate
g = []          # Constraints
ub = []         # Upper bound
lb = []         # Lower bound

# 3X3 MATRICES EIGENVALUES COMPUTATION  VIA POWER ITERATION METHOD
def powerm(v, A, niter): #POWER ITERATION METHOD
    for i in range(niter):
        v = A @ v
        v = v/casadi.norm_2(v)
    l = v.T@A@v
    return v, l
def def_powerm(a): #DEFLATION VERSION OF PIM FOR 3X3 MATRIX ---> CALCULATE ALL EIGENVALUES FROM GREATER TO SMALLER
    #AND THEIR EIGENVECTORS
    v0 = casadi.DM([1,1,1])
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

for i in range(thunder_ahand.get_numJoints()):
    # Mass must be positive
    g.append(hat_Pi_sym[i*10])
    lb.append(0)
    ub.append(np.inf)
    # Semi-definite inertia matrix
    Ixx = hat_Pi_sym[i*10 + 4]
    Ixy = hat_Pi_sym[i*10 + 5]
    Ixz = hat_Pi_sym[i*10 + 6]
    Iyy = hat_Pi_sym[i*10 + 7]
    Iyz = hat_Pi_sym[i*10 + 8]
    Izz = hat_Pi_sym[i*10 + 9]
    Ib = np.array([[Ixx, Ixy, Ixz],
                   [Ixy, Iyy, Iyz],
                   [Ixz, Iyz, Izz]])
    _,l = def_powerm(Ib)
    g.append((Ib[0,0]+Ib[1,1]+Ib[2,2])/2 - l[0])
    lb.append(0)
    ub.append(np.inf)


prob = {}
prob['x'] = hat_Pi_sym
prob['f'] = (YY@hat_Pi_sym - TTau).T@(YY@hat_Pi_sym - TTau)
prob['g'] = casadi.vertcat(*g)
solver = casadi.nlpsol('sol', 'ipopt', prob)
sol = solver(x0=hat_Pi, lbg=lb,ubg=ub)
hat_Pi = sol['x'].full()

print("Residual norm: ", np.linalg.norm(YY@hat_Pi - TTau))
print("=== Dynamic parameters ======")
print("PI size:", hat_Pi.shape)
for i in range(thunder_ahand.get_numJoints()):
    print(f"\tlink {i}:")
    print(f"\t\tinertial:")
    print(f"\t\t\tsymb: [1,1,1,1,1,1,1,1,1,1]")
    print(f"\t\t\tmass: {hat_Pi[i*10]}")
    print(f"\t\t\tCoM_x: {hat_Pi[i*10 + 1]}")
    print(f"\t\t\tCoM_y: {hat_Pi[i*10 + 2]}")
    print(f"\t\t\tCoM_z: {hat_Pi[i*10 + 3]}")
    print(f"\t\t\tIxx: {hat_Pi[i*10 + 4]}")
    print(f"\t\t\tIxy: {hat_Pi[i*10 + 5]}")
    print(f"\t\t\tIxz: {hat_Pi[i*10 + 6]}")
    print(f"\t\t\tIyx: {hat_Pi[i*10 + 7]}")
    print(f"\t\t\tIyy: {hat_Pi[i*10 + 8]}")
    print(f"\t\t\tIzz: {hat_Pi[i*10 + 9]}")
print("=============================")

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
    Y = thunder_ahand.get_reg_M() + thunder_ahand.get_reg_C() + thunder_ahand.get_reg_G()
    tau_est = np.array(Y @ hat_Pi).flatten()
    delta_tau.append(tau_ctr_log[i,:] - tau_est)
delta_tau = np.array(delta_tau)

print("> Making Plot")
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
# --- 1. Plot joint trajectories ---
plt.figure(figsize=(12,8))
for i in range(4):
    plt.subplot(2,2,i+1)
    plt.plot(t_log, qd_log[:,i], 'b', label='dq')# Plot joint limits
    plt.title(f'Joint {joint_indices[i]}')
    plt.grid(True)
    if i==0:
        plt.legend()
plt.tight_layout()
plt.show(block=False)
plt.figure(figsize=(12,8))
for i in range(4):
    plt.subplot(2,2,i+1)
    plt.plot(t_log, qdd_log[:,i], 'b', label='ddq')# Plot joint limits
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
    plt.plot(t_log, tau_ctr_log[:,i]-delta_tau[:,i], linestyle='dotted', label = 'hat_tau')
    plt.xlabel('Time [s]')
    plt.ylabel('Torque [Nm]')
    plt.title(f'Joint {joint_indices[i]}')
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
    plt.title(f'Joint {joint_indices[i]}')
    plt.grid(True)
    if i==0:
        plt.legend()
plt.tight_layout()
plt.show()

#exit(0)