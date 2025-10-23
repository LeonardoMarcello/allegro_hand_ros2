import time
import os
import matplotlib.pyplot as plt


import mujoco
import mujoco.viewer
import numpy as np

from AHAND import AHAND, MESH_DIR, URDF_PATH, XML_PATH

# --- Storage arrays ---
t_log = []
q_log = []
qd_log = []
pin_c_log = []
pin_grav_log = []
grav_log = []
tau_log = []
tau_ctr_log = []
q_ref_log = []
qd_ref_log = []
qdd_ref_log = []
e_log = []


if not os.path.exists(XML_PATH):
  print("Loading existing URDF file")
  m = mujoco.MjModel.from_xml_path(URDF_PATH)
  mujoco.mj_saveLastXML(XML_PATH, m)
  print("Exported URDF to XML file")
  exit(0)


m = mujoco.MjModel.from_xml_path(XML_PATH)
d = mujoco.MjData(m)

ahand = AHAND(URDF_PATH, MESH_DIR)
ahand.initGepettoViewer()
ahand.show()

q_rnd = ahand.getRandomConfig()
q_rnd[13] = 0 # otherwise start with contact

q_rnd = (ahand.JOINT_LIMITS[:,1] + ahand.JOINT_LIMITS[:,0])/2

ahand.q = q_rnd.copy() # <-- I-T-M-R
ahand.updateConfig()

d.qpos = q_rnd[[0,1,2,3,   8,9,10,11,   12,13,14,15,   4,5,6,7]].copy()  # <-- I-M-R-T
# Parameters
# Trajectory: sinusoidal motion for testing
amplitude = np.pi/4*np.array([0.2, 0.3, 0.4, 0.5,   0.2, 0.3, 0.4, 0.5,   0.2, 0.3, 0.4, 0.5,   0.2, 0.3, 0.4, 0.5])  # radians
frequency = 0.5*np.array([0.2, 0.7, 0.6, 0.8,   0.2, 0.7, 0.6, 0.8,   0.2, 0.7, 0.6, 0.8,   0.2, 0.7, 0.6, 0.8])  # Hz

q_traj = np.zeros((16,))
qd_traj = np.zeros_like(q_traj)
qdd_traj = np.zeros_like(q_traj)
tau = np.zeros_like(q_traj)

e = np.zeros((16,))
de = np.zeros((16,))
integral_e = np.zeros((16,))

with mujoco.viewer.launch_passive(m, d) as viewer:
  # Close the viewer automatically after 30 wall-seconds.
  start = time.time()
  while viewer.is_running() and time.time() - start < 60:
    step_start = time.time()


    # mj_step can be replaced with code that also evaluates
    # a policy and applies a control signal before stepping the physics.
    t = step_start - start

    for i in range(16):
      q_traj[i] = amplitude[i] * np.sin(2 * np.pi * frequency[i] * t) + q_rnd[i]  # <-- I-T-M-R
      qd_traj[i] = 2 * np.pi * frequency[i] * amplitude[i] * np.cos(2 * np.pi * frequency[i] * t)
      qdd_traj[i] = - (2 * np.pi * frequency[i])**2 * amplitude[i] * np.sin(2 * np.pi * frequency[i] * t)


    de = (e - (q_traj - d.qpos[[0,1,2,3,   12,13,14,15,   4,5,6,7,   8,9,10,11]]))/m.opt.timestep     # <-- I-T-M-R
    e = q_traj - d.qpos[[0,1,2,3,   12,13,14,15,   4,5,6,7,   8,9,10,11]]
    integral_e += e*m.opt.timestep
    kp = 70*np.diag([1.5,2.0,1.8,1.3,    1.5,2.0,1.8,1.3,    1.5,2.0,1.8,1.3,    2.2,1.7,1.9,1.3])
    kd = np.diag([0.1,0.11,0.12,0.13, 0.1,0.11,0.12,0.13, 0.1,0.11,0.12,0.13, 0.15,0.14,0.12,0.11])
    ki = 20*np.diag([1,1,1,1,    1,1,1,1,  1,1,1,1,  1,1,1,1])



    # State:           Index                Thumb               Middle                  Ring
    # q =       [q_0,q_1,q_2,q_3,      q_12,q_13,q_14,q_15     q_4,q_5,q_6,q_7,        q_8,q_9,q_10,q_11]

    # State:           Index                Middle                  Ring                Thumb
    # d =       [q_0,q_1,q_2,q_3,        q_4,q_5,q_6,q_7,        q_8,q_9,q_10,q_11   q_12,q_13,q_14,q_15]

    #d.ctrl = tau[[0,1,2,3,   8,9,10,11,   12,13,14,15,   4,5,6,7]]

    g = ahand.getGravityVector()                            # <-- I-T-M-R

    b = ahand.getDynamicDrift()                             # <-- I-T-M-R
    c = b-g

    M = ahand.getMassMatrix()                               # <-- I-T-M-R

    # Computed Torque
    #tau = M @ (qdd_traj[[0,1,2,3,    12,13,14,15,   4,5,6,7,      8,9,10,11]] + 1*e + 0.1*de + 1*integral_e) + C
    #tau = M @ qdd_traj + b + 3*np.eye(16) @ ahand.dq        # <-- I-T-M-R
    tau = g         # <-- I-T-M-R
    
    # PID
    #tau = kp@e + ki@integral_e        # <-- I-T-M-R
    #tau = np.zeros_like(ahand.q)

    g = g[[0,1,2,3,   8,9,10,11,   12,13,14,15,   4,5,6,7]]               # <-- I-M-R-T
    b = b[[0,1,2,3,   8,9,10,11,   12,13,14,15,   4,5,6,7]]               # <-- I-M-R-T
    c = c[[0,1,2,3,   8,9,10,11,   12,13,14,15,   4,5,6,7]]               # <-- I-M-R-T

    d.ctrl = tau[[0,1,2,3,   8,9,10,11,   12,13,14,15,   4,5,6,7]]        # <-- I-M-R-T


    # ========== Sim Step ===============
    mujoco.mj_step(m, d)
    # ===================================

    # tau = d.qfrc_passive + d.qfrc_actuator + d.qfrc_applied
    #     d.qfrc_passive: includes passive forces from spring-dampers and fluid dynamics
    #     d.qfrc_actuator: actuation forces
    #     d.qfrc_applied: additional forces specified by the user
    # b = d.qfrc_bias (Coriolis, Centrifugal and Gravity)

    # update AHAND model to match MuJoCo state
    ahand.set_index(d.qpos[0:4], d.qvel[0:4], d.qacc[0:4])
    ahand.set_middle(d.qpos[4:8], d.qvel[4:8], d.qacc[4:8])
    ahand.set_ring(d.qpos[8:12], d.qvel[8:12], d.qacc[8:12])
    ahand.set_thumb(d.qpos[12:16], d.qvel[12:16], d.qacc[12:16])
    ahand.set_target(q_traj)
    ahand.updateConfig()
    ahand.show(target=True)

    # Example modification of a viewer option: toggle contact points every two seconds.
    with viewer.lock():
      viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)

    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()

    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)

          # --- Log data ---
    t_log.append(t)
    q_log.append(d.qpos[[0,1,2,3,   12,13,14,15,   4,5,6,7,   8,9,10,11]].copy())                   # <-- I-T-M-R
    qd_log.append(d.qvel[[0,1,2,3,  12,13,14,15,   4,5,6,7,   8,9,10,11]].copy())                   # <-- I-T-M-R
    pin_c_log.append(c[[0,1,2,3,  12,13,14,15,   4,5,6,7,   8,9,10,11]].copy())                     # <-- I-T-M-R
    pin_grav_log.append(g[[0,1,2,3,  12,13,14,15,   4,5,6,7,   8,9,10,11]].copy())                  # <-- I-T-M-R
    grav_log.append(d.qfrc_bias[[0,1,2,3,  12,13,14,15,   4,5,6,7,   8,9,10,11]].copy())            # <-- I-T-M-R
    tau_log.append(d.qfrc_constraint[[0,1,2,3,  12,13,14,15,   4,5,6,7,   8,9,10,11]].copy() +
                   d.qfrc_smooth[[0,1,2,3,  12,13,14,15,   4,5,6,7,   8,9,10,11]].copy())           # <-- I-T-M-R
    tau_ctr_log.append(tau.copy())                                                  # <-- I-T-M-R
    q_ref_log.append(q_traj.copy())                                                 # <-- I-T-M-R
    qd_ref_log.append(qd_traj.copy())                                               # <-- I-T-M-R
    qdd_ref_log.append(qdd_traj.copy())                                             # <-- I-T-M-R
    e_log.append(e.copy())

  print("Making Plot")
# --- Convert logs to numpy arrays ---
t_log = np.array(t_log)
q_log = np.array(q_log)
qd_log = np.array(qd_log)
grav_log = np.array(grav_log)
pin_c_log = np.array(pin_c_log)
pin_grav_log = np.array(pin_grav_log)
tau_log = np.array(tau_log)
tau_ctr_log = np.array(tau_ctr_log)
q_ref_log = np.array(q_ref_log)
qd_ref_log = np.array(qd_ref_log)
qdd_ref_log = np.array(qdd_ref_log)
e_log = np.array(e_log)

# --- 1. Plot joint trajectories ---
plt.figure(figsize=(12,8))
for i in range(16):
    plt.subplot(4,4,i+1)
    plt.plot(t_log, q_ref_log[:,i], 'r--', label='q_ref')
    plt.plot(t_log, q_log[:,i], 'b', label='q')# Plot joint limits
    plt.hlines(ahand.JOINT_LIMITS[i, 0], t_log[0], t_log[-1], colors='k', linestyles='dotted', linewidth=1)
    plt.hlines(ahand.JOINT_LIMITS[i, 1], t_log[0], t_log[-1], colors='k', linestyles='dotted', linewidth=1)
    plt.title(f'Joint {i}')
    plt.grid(True)
    if i==0:
        plt.legend()
plt.tight_layout()
plt.show(block=False)
# --- 2. Plot joint trajectories ---
plt.figure(figsize=(12,8))
for i in range(16):
    plt.subplot(4,4,i+1)
    plt.plot(t_log, qd_ref_log[:,i], 'r--', label='dq_ref')
    plt.plot(t_log, qd_log[:,i], 'b', label='dq')
    plt.title(f'Joint {i}')
    plt.grid(True)
    if i==0:
        plt.legend()
plt.tight_layout()
plt.show(block=False)

# --- 3. Plot Drift ---
plt.figure(figsize=(12,8))
for i in range(16):
    plt.subplot(4,4,i+1)
    plt.plot(t_log, pin_c_log[:,i], 'b', label='c')
    plt.xlabel('Time [s]')
    plt.ylabel('Torque [Nm]')
    plt.title(f'Joint {i}')
    plt.grid(True)
    if i==0:
        plt.legend()
plt.tight_layout()
plt.show(block=False)
# --- 4. Plot Gravity ---
plt.figure(figsize=(12,8))
for i in range(16):
    plt.subplot(4,4,i+1)
    plt.plot(t_log, pin_grav_log[:,i], 'r--', label='g')
    plt.plot(t_log, grav_log[:,i], 'b', label='mjc_g')
    plt.xlabel('Time [s]')
    plt.ylabel('Torque [Nm]')
    plt.title(f'Joint {i}')
    plt.grid(True)
    if i==0:
        plt.legend()
plt.tight_layout()
plt.show(block=False)

# --- 5. Plot torques ---
plt.figure(figsize=(12,8))
for i in range(16):
    plt.subplot(4,4,i+1)
    plt.plot(t_log, tau_log[:,i], label = 'tau')
    plt.xlabel('Time [s]')
    plt.ylabel('Torque [Nm]')
    plt.title(f'Joint {i}')
    plt.grid(True)
    if i==0:
        plt.legend()
plt.tight_layout()
plt.show(block=False)

# --- 6. Plot input ---
plt.figure(figsize=(12,8))
for i in range(16):
    plt.subplot(4,4,i+1)
    plt.plot(t_log, tau_ctr_log[:,i], label='ctrl')
    plt.xlabel('Time [s]')
    plt.ylabel('Torque [Nm]')
    plt.title(f'Joint {i}')
    plt.grid(True)
    if i==0:
        plt.legend()
plt.tight_layout()
plt.show(block=False)

# --- 7. Plot errors ---
plt.figure(figsize=(12,6))
for i in range(16):
    plt.plot(t_log, e_log[:,i], label=f'Joint {i}')
plt.xlabel('Time [s]')
plt.ylabel('Error [rad]')
plt.title('Tracking Errors')
plt.legend(ncol=4, fontsize=8)
plt.grid(True)
plt.show()
#exit(0)