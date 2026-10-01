import gymnasium
import mujoco_env
import numpy as np

import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import utils.import_thunder as thunder

from gymnasium.envs.registration import register

# Register and create environment
register(
    id='mujoco_env/MyEnv-v0',
    entry_point='mujoco_env.envs.my_env:MyEnv',  # <-- YOU NEED TO CHANGE THIS LINE
    #max_episode_steps=1000,                # Optional: specify when the episode truncates
)
env = gymnasium.make('mujoco_env/MyEnv-v0', render_mode='human')

# Configure thunder hand model
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


# 1. Load your CSV data
import pandas as pd
import matplotlib.pyplot as plt
# 3. Setup Plotting
plt.ion() # Turn on interactive mode
fig, axs = plt.subplots(2, 2, figsize=(10, 8))
fig.suptitle('Finger 1: CSV vs Simulation Joint Positions')
axs = axs.flatten()

# Lines for plotting (recorded = dashed, sim = solid)
line_rec = [axs[i].plot([], [], 'r--', label='Recorded')[0] for i in range(4)]
line_sim = [axs[i].plot([], [], 'b-', label='Simulation')[0] for i in range(4)]

# Data buffers for the plot
plot_time = []
plot_rec_q = [[] for _ in range(4)]
plot_sim_q = [[] for _ in range(4)]

for i, ax in enumerate(axs):
    ax.set_title(f'Joint {i}')
    ax.legend(loc='upper right')
    ax.grid(True)
# Assuming 'torques.csv' has columns: [timestamp, tau0, tau1, ..., tau15]
df_tau = pd.read_csv('/home/leo/Desktop/allegro_hand_ros2_ws/Python/v2/data/finger_identif_traject/tau_identification.csv')
df_q = pd.read_csv('/home/leo/Desktop/allegro_hand_ros2_ws/Python/v2/data/finger_identif_traject/q_identification.csv')
df_t = pd.read_csv('/home/leo/Desktop/allegro_hand_ros2_ws/Python/v2/data/finger_identif_traject/t_identification.csv')
recorded_timestamps = df_t.iloc[:, 0].values    # time column
recorded_torques = df_tau.iloc[:, :].values     # torque columns
recorded_joints = df_q.iloc[:, :].values     # joint columns
# Normalize timestamps so t0 = 0
t0 = recorded_timestamps[0]
relative_recorded_times = recorded_timestamps - t0
# 2. Setup tracking variables
csv_ptr = 0
num_samples = len(relative_recorded_times)
sim_time = 0.0  # Internal simulation clock
history_time = []
history_sim_q = []  # To store simulation joint positions
history_rec_q = []  # To store CSV joint positions
# Total steps to run (based on CSV length)
max_steps = num_samples

dt = env.unwrapped._timestep


# # Change gravity due to hand orientation (hard coded)
from scipy.spatial.transform import Rotation as R
g = np.array([0,0,-9.1])
# Rot = R.from_euler('xyz', [0, -np.pi/2, 0])
# new_g = Rot.as_matrix().T@g.T
# print(f"timestep {env.unwrapped._timestep}")
# for i in range(4):
#     fingers[i].set_par_gravity(new_g)

# Reset the environment with a specific seed for reproducibility
observation, info = env.reset(seed=42)
tau = np.zeros((16,))

#Cube Sides
des_contact_points = np.array([ [-1, 0, 0],
                                [0, -1, 0],
                                [1, 0, 0],
                                [0, 1, 0],])
half_extent = 0.02+0.01#0.03


# (Add these tuning parameters outside the loop if you prefer)
d_safe = 0.03  # Distance threshold to start repelling (e.g., 3cm from surface)
k_rep = 0.005  # Repulsive gain (tune this based on your scale/dt)

t = 0
s = 0

x_final = np.array([0,0,0])
while True:

    # ... [your existing observation parsing] ...
    x_obj = observation['object_state']['x']
    quat_obj = observation['object_state']['quat']
    rotation = R.from_quat(quat_obj)
    # set current state
    for i in range(4):
        fingers[i].set_q(observation['joint_states']['q'][4*i:4*(i+1)])
        fingers[i].set_dq(observation['joint_states']['dq'][4*i:4*(i+1)])

        # 1. Attractive term (Original desired translation)
        if s>4000:
            x_des = x_final.copy() + np.array([0.02,0,0]) # lift up
        else:
            x_des = x_obj + rotation.apply(half_extent*des_contact_points[i,:])
            x_final = x_des.copy()

        x_ee = fingers[i].get_T_w_ee()[0:3,3]
        e_att = x_des - x_ee

        # 2. Repulsive term calculations
        v_repulsive = np.zeros(3)
        v_obj_to_ee = x_ee - x_obj
        dist_to_center = np.linalg.norm(v_obj_to_ee)

        # Approximate distance to surface using an inscribed sphere 
        # (Smooth and usually sufficient for approach phase)
        dist_to_surface = dist_to_center - half_extent

        # If the finger is inside the safe zone, trigger repulsion
        if 0 < dist_to_surface < d_safe:
            # Direction: push outward from the center
            dir_repulsive = v_obj_to_ee / dist_to_center

            # Magnitude calculation
            dist_to_target_sq = np.dot(e_att, e_att)

            # Formula: k * (1/d - 1/d0) * (1/d^2) * ||e_att||^2
            mag_repulsive = k_rep * (1.0/dist_to_surface - 1.0/d_safe) * (1.0/(dist_to_surface**2)) * dist_to_target_sq

            v_repulsive = dir_repulsive * mag_repulsive

        # 3. Combine attractive and repulsive vectors
        if t<4000:
            t+=1
            e_total = e_att + v_repulsive
        else:
            s+=1
            e_total = e_att
        # Inverse Kinematics using the combined vector
        J = fingers[i].get_J_ee()[:3,:]
        #det = np.sqrt(np.linalg.det(J.T@J))
        # if np.isclose(det, 0.0, atol=1e-16):
        #     print(f"WARN: J of finger {i} is close to singularity")
        N = (np.eye(4) - np.linalg.pinv(J).dot(J))

        # Replaced 'e' with 'e_total' here

        dq = np.linalg.pinv(J).dot(e_total) - N@fingers[i].get_q()
        dt = env.unwrapped._timestep
        
        if i == 2 or i == 0:
            kp = kv = 0
        else:
            kp = 800 
            kv = 1
            
        tau[4*i:4*(i+1)] = kp*(dq*dt) - kv*fingers[i].get_dq() + fingers[i].get_G()
        #tau[4*i:4*(i+1)] = J.T @ (kp*(e_att) - kv*J@fingers[i].get_dq()) + fingers[i].get_G()
        #tau[4*i:4*(i+1)] = fingers[i].get_G()

    # Compute control torque
    # kp = 5
    # kv = 1
    # target = np.array([np.pi/4, np.pi/4, np.pi/3, np.pi/2])
    # tau = np.hstack([fingers[0].get_G() + kp*(target - fingers[0].get_q())- kv*fingers[0].get_dq(), 
    #                   fingers[1].get_G() + kp*(target - fingers[1].get_q())- kv*fingers[1].get_dq(), 
    #                   fingers[2].get_G() + kp*(target - fingers[2].get_q())- kv*fingers[2].get_dq(), 
    #                   fingers[3].get_G() + kp*(target - fingers[3].get_q())- kv*fingers[3].get_dq()])
    tau_g = np.hstack([fingers[0].get_G(), 
                     fingers[1].get_G(), 
                     fingers[2].get_G(), 
                     fingers[3].get_G()])

    # Load CSV torque data
    if csv_ptr >= num_samples - 1:
        print("End of CSV sequence reached.")
    while csv_ptr < num_samples - 1 and sim_time >= relative_recorded_times[csv_ptr + 1]:
        csv_ptr += 1
    # Get the torque for this synchronized step
    #tau_g[4:8] = recorded_torques[csv_ptr]


    #tau = np.zeros_like(tau)



    # Take a step in the environment using the chosen action
    observation, reward, terminated, truncated, info = env.step(tau_g)
    sim_time += dt

    # Record data for plotting
    history_time.append(sim_time)
    history_sim_q.append(observation['joint_states']['q'][4:8].copy())
    history_rec_q.append(recorded_joints[csv_ptr].copy())
    # 2. Plotting Section (Executed only after loop finishes)
    if sim_time>60:
        history_sim_q = np.array(history_sim_q)
        history_rec_q = np.array(history_rec_q)

        fig, axs = plt.subplots(2, 2, figsize=(12, 8))
        fig.suptitle('Finger 1: Joint Position Tracking (Sim vs CSV)', fontsize=16)
        joint_names = ['Base', 'Proximal', 'Middle', 'Distal']

        for i in range(4):
            row, col = divmod(i, 2)
            axs[row, col].plot(history_time, history_rec_q[:, i], 'r--', label='Recorded (CSV)', alpha=0.8)
            axs[row, col].plot(history_time, history_sim_q[:, i], 'b-', label='Simulation', alpha=0.8)
            
            axs[row, col].set_title(f'Joint {i} ({joint_names[i]})')
            axs[row, col].set_xlabel('Time [s]')
            axs[row, col].set_ylabel('Position [rad]')
            axs[row, col].legend()
            axs[row, col].grid(True, linestyle=':', alpha=0.6)

        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        plt.show()
    # Update the live plot
    #env.unwrapped.plot()

    # # Check if the episode is over (terminated) or max steps reached (truncated)
    # if terminated or truncated:
    #     # If the episode ends or is truncated, reset the environment
    #     observation, info = env.reset()

# Close the environment when the simulation is done
env.close()








