import time
import os
import matplotlib.pyplot as plt


import mujoco
import mujoco.viewer
import numpy as np

import utils.import_thunder as thunder

index = thunder.thunder_ahand_finger()
middle = thunder.thunder_ahand_finger()
ring = thunder.thunder_ahand_finger()
thumb = thunder.thunder_ahand_thumb()

# Load config
thunder.load_params(index, "/home/leo/Desktop/allegro_hand_ros2_ws/Python/v2/config/ahand_index_identification_par.yaml")
thunder.load_params(middle, "/home/leo/Desktop/allegro_hand_ros2_ws/Python/v2/config/ahand_middle_identification_par.yaml")
thunder.load_params(ring, "/home/leo/Desktop/allegro_hand_ros2_ws/Python/v2/config/ahand_ring_identification_par.yaml")
thunder.load_params(thumb, "/home/leo/Desktop/allegro_hand_ros2_ws/Python/v2/config/ahand_thumb_identification_par.yaml")
fingers = [index, middle, ring, thumb]


# Finger Frames conversion
T_1_DH2URDF = np.eye(4)
T_1_DH2URDF[2,3] = -0.017
T_2_DH2URDF = np.array([[0, 1, 0, 0],
                        [0, 0, 1, 0],
                        [1, 0, 0, 0],
                        [0, 0, 0, 1],])
T_3_DH2URDF = T_2_DH2URDF.copy()
T_4_DH2URDF = T_2_DH2URDF.copy()
T_finger_transforms = [T_1_DH2URDF.copy(),T_2_DH2URDF.copy(),T_3_DH2URDF.copy(),T_4_DH2URDF.copy()]
# Thumb Frames conversion
T_1_DH2URDF = np.array([[0, 0, -1, 0],
                        [-1, 0, 0, 0],
                        [0, 1, 0, 0],
                        [0, 0, 0, 1]])

T_2_DH2URDF = np.array([[-1,  0, 0, 0],
                        [0, -1, 0, 0],
                        [0,  0, 1, 0],
                        [0,  0, 0, 1]])

T_3_DH2URDF = np.array([[0,  1,  0, 0],
                        [0,  0,  1, 0],
                        [1,  0,  0, 0],
                        [0,  0,  0, 1]])
T_4_DH2URDF = np.array([[0,  1,  0, 0],
                        [0,  0,  1, 0],
                        [1,  0,  0, 0],
                        [0,  0,  0, 1]])

T_thumb_transforms = [T_1_DH2URDF.copy(),T_2_DH2URDF.copy(),T_3_DH2URDF.copy(),T_4_DH2URDF.copy()]

import xml.etree.ElementTree as ET
tree = ET.parse("/home/leo/Desktop/allegro_hand_ros2_ws/Python/v2/assets/allegro_hand_v5/urdf/allegro_hand_v5_leo_tmp.xml")
root = tree.getroot()
for i in range(4): # num of fingers
  finger_dyn = fingers[i].get_par_DYN()
  finger_damping = fingers[i].get_par_Dl()[1::2]
  finger_frictionloss = fingers[i].get_par_Dl()[0::2]
  finger_armature = fingers[i].get_par_Ia()
  for j in range(4): #link per finger
    # Calculate the exact link and joint index (e.g., link_12_0)
    body_name = f"link_{4*i+j}_0"
    joint_name = f"joint_{4*i+j}_0"

    # Extract parameters for this specific link (i-th link in the chain)
    dyn = finger_dyn[j*10:(j+1)*10]
    mass = dyn[0]
    # com = dyn[1:4]
    # # Convert C++ output [Ixx, Ixy, Ixz, Iyy, Iyz, Izz] 
    # # to MuJoCo format   [Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
    # full_inertia = [dyn[4], dyn[7], dyn[9], dyn[5], dyn[6], dyn[8]]

    # 1. Get the correct transformation matrix for the current link
    if i < 3:
      T = T_finger_transforms[j]
    elif i==3:
      T = T_thumb_transforms[j]


    R = T[:3, :3]
    trans = T[:3, 3]

    # 2. Reconstruct the 3x3 Inertia Tensor at the DH origin
    # (Standard dynamic parameters: Ixx, Ixy, Ixz, Iyy, Iyz, Izz)
    I_DH = np.array([
        [dyn[4], dyn[5], dyn[6]],
        [dyn[5], dyn[7], dyn[8]],
        [dyn[6], dyn[8], dyn[9]]
    ])
    # 5. Rotate the Inertia into the URDF frame
    I_urdf = R @ I_DH @ R.T
    # Pack back into MuJoCo's expected order: [Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
    full_inertia = [
        I_urdf[0,0], I_urdf[1,1], I_urdf[2,2],
        I_urdf[0,1], I_urdf[0,2], I_urdf[1,2]
    ]

    # 6. Transform the CoM to the URDF frame
    cx, cy, cz = dyn[1], dyn[2], dyn[3]
    com_urdf = R @ np.array([cx, cy, cz]) + trans
    # 7. Override the original variables so your XML writer uses the converted values
    com = com_urdf





    damping = finger_damping[j]
    frictionloss = finger_frictionloss[j]
    armature = finger_armature[j]

    # --- Update the Link (Body -> Inertial) ---
    body = root.find(f".//body[@name='{body_name}']")
    if body is not None:
        inertial = body.find("inertial")
        if inertial is not None:
            inertial.set("mass", f"{mass:.6g}")
            inertial.set("pos", f"{com[0]:.6g} {com[1]:.6g} {com[2]:.6g}")
            inertial.set("fullinertia", " ".join([f"{val:.6g}" for val in full_inertia]))
            
            # Remove old diaginertia and quat to prevent MuJoCo errors
            inertial.attrib.pop("diaginertia", None)
            inertial.attrib.pop("quat", None)

    # --- Update the Joint ---
    joint = root.find(f".//body[@name='{body_name}']/joint[@name='{joint_name}']")
    if joint is not None:
        joint.set("damping", f"{damping:.6g}")
        joint.set("frictionloss", f"{frictionloss:.6g}")
        joint.set("armature", f"{armature:.6g}")
tree.write("/home/leo/Desktop/allegro_hand_ros2_ws/Python/v2/assets/allegro_hand_v5/urdf/allegro_hand_v5_leo_tmp.xml", encoding="utf-8", xml_declaration=True)
print("xml Exported")

from utils.AHAND import AHAND, MESH_DIR, URDF_PATH, XML_PATH
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

#XML_PATH = "/home/leo/Desktop/allegro_hand_ros2_ws/Python/v2/description/wonik_allegro/right_hand.xml"

if not os.path.exists(XML_PATH):
  print("Loading existing URDF file")
  m = mujoco.MjModel.from_xml_path(URDF_PATH)
  mujoco.mj_saveLastXML(XML_PATH, m)
  print("Exported URDF to XML file")
  exit(0)

m = mujoco.MjModel.from_xml_path(XML_PATH)
d = mujoco.MjData(m)

ahand = AHAND(URDF_PATH, MESH_DIR)
#ahand.initGepettoViewer()
#ahand.show()

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
  while viewer.is_running() and time.time() - start < 20:
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
    #tau = M @ (qdd_traj[[0,1,2,3,    12,13,14,15,   4,5,6,7,      8,9,10,11]] + 30*e + 3*de + 30*integral_e) + c
    #tau = M @ qdd_traj + b + 3*np.eye(16) @ ahand.dq        # <-- I-T-M-R
    #tau = g         # <-- I-T-M-R
    
    # PID
    tau = kp@e + ki@integral_e + g       # <-- I-T-M-R
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
    #ahand.show(target=True)

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