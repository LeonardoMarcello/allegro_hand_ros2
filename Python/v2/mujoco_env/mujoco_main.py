import gymnasium
import mujoco_env
import numpy as np

import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))


from gymnasium.envs.registration import register
# Register and create environment
register(
    id='mujoco_env/MyEnv-v0',
    entry_point='mujoco_env.envs.my_env:MyEnv',  # <-- YOU NEED TO CHANGE THIS LINE
    #max_episode_steps=1000,                # Optional: specify when the episode truncates
)
env = gymnasium.make('mujoco_env/MyEnv-v0', render_mode='human')


import utils.import_thunder as thunder
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

# Init Controller
from mujoco_env.controllers import CartesianImpedanceController
controller = CartesianImpedanceController(fingers, np.zeros((12,)))

# Plot
from mujoco_env.plotting.live_plot import LivePlot
plotter = LivePlot(timestep=0.01, update_freq_hz=10)
counter = 0
plotter.start()


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
hand2world_d = [0.15,0,0.15]
hand2world_R = R.from_quat([0.7071068, 0, 0, -0.7071068])

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


x_des = np.zeros((12))
x_des[3:6] = [0.15, 0.1 - 0.025 - 0.013 , 0.2]



e_rmse = 0

while True:

    # Manipulated object
    x_obj = observation['object_state']['x']
    quat_obj = observation['object_state']['quat']
    rotation = R.from_quat(quat_obj)

    # set controller
    current_joint_states = observation['joint_states']
    controller.set_current_state(current_joint_states)

    x_des[:3] = env.unwrapped.get_desired_ee('index')
    x_des[3:6] = env.unwrapped.get_desired_ee('middle')
    x_des[6:9] = env.unwrapped.get_desired_ee('ring')
    x_des[9:] = env.unwrapped.get_desired_ee('thumb')
    controller.set_target(x_des)

    dt = env.unwrapped._timestep
    tau = controller.compute(dt)

    # set current state
    # for i in range(4):
    #     fingers[i].set_q(observation['joint_states']['q'][4*i:4*(i+1)])
    #     fingers[i].set_dq(observation['joint_states']['dq'][4*i:4*(i+1)])


    #     x_des = env.unwrapped.get_desired_ee('middle')
    #     # x_des_inhand = x_des + rotation.apply(half_extent*des_contact_points[i,:])
    #     x_ee = fingers[i].get_T_w_ee()[0:3,3]
    #     e_ee = x_des - x_ee
    #     # Inverse Kinematics using the combined vector
    #     J = fingers[i].get_J_ee()[:3,:]
    #     N = (np.eye(4) - np.linalg.pinv(J).dot(J))
    #     # Replaced 'e' with 'e_total' here
    #     dq = np.linalg.pinv(J).dot(e_ee) - N@fingers[i].get_q()
    #     dt = env.unwrapped._timestep

    #     # joint level control
    #     # if i==1:
    #     #     q_e = recorded_joints[csv_ptr].copy() - observation['joint_states']['q'][4:8].copy()
    #     #     e_rmse = np.sqrt(np.sum(q_e**2)/4)
    #     #     print(e_rmse)
    #     # else:
    #     #     q_e = np.array([0,0,0,0])
    #     # dt = env.unwrapped._timestep
    #     # kp = 1
    #     # kv = 0.1
    #     # tau[4*i:4*(i+1)] = kp*(q_e) - kv*fingers[i].get_dq() + fingers[i].get_G()

    #     if i==1:
    #         kp = 800 
    #         kv = 1
    #         tau[4*i:4*(i+1)] = kp*(dq*dt) - kv*fingers[i].get_dq() + fingers[i].get_G()
    #     else:
    #         tau[4*i:4*(i+1)] = fingers[i].get_G()

    # Take a step in the environment using the chosen action
    observation, reward, terminated, truncated, info = env.step(tau)
    sim_time += dt

    if counter % 10 == 0:
        # Send data to the background thread instantly without blocking
        plotter.update(observation, sim_time) 
    counter += 1
    
    
    sim_time += dt
    # Load CSV torque data
    if csv_ptr >= num_samples - 1:
        print("End of CSV sequence reached.")
    while csv_ptr < num_samples - 1 and sim_time >= relative_recorded_times[csv_ptr + 1]:
        csv_ptr += 1

    # Update the live plot
    #env.unwrapped.plot()

    # # Check if the episode is over (terminated) or max steps reached (truncated)
    # if terminated or truncated:
    #     # If the episode ends or is truncated, reset the environment
    #     observation, info = env.reset()

# Close the environment when the simulation is done
env.close()








