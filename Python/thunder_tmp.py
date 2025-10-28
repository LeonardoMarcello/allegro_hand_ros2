import numpy as np
import matplotlib.pyplot as plt
import sys
sys.path.append("/home/leo/thunder_dynamics/ahand_finger_generatedFiles_py/build") # Where the .so file is located. 
sys.path.append("/home/leo/thunder_dynamics/franka_as_generatedFiles/build") # Where the .so file is located. 
# Note: This is not needed if the .so file is in the same directory as the python script

from thunder_ahand_finger_py import thunder_ahand_finger
from thunder_franka_as_py import thunder_franka_as

# Load the robot model
# ---------------------------------------------------------------------------
robot = thunder_ahand_finger()
robot.load_conf("/home/leo/thunder_dynamics/ahand_finger_generatedFiles/ahand_finger_conf.yaml")

#robot = thunder_franka_as()
#robot.load_conf("/home/leo/thunder_dynamics/franka_as_generatedFiles/franka_as_conf.yaml")

print("Number of joints: ", robot.get_numJoints())
curr_q = np.array([0,0, 0,0], dtype=float)
robot.set_q(np.random.rand(robot.get_numJoints()))
robot.set_q(curr_q)
#print("Current joint positions: ", robot.get_q())
dq = np.random.rand(robot.get_numJoints())
robot.set_dq(dq)
robot.set_dqr(dq)
#robot.set_dq(np.zeros(robot.get_numJoints()))
#print("Current joint positions: ", robot.get_q())





print("End-effector pose: \n", robot.get_T_0_ee())


J = robot.get_J_ee()
M = robot.get_M()
print("Mass Matrix: \n", M)
C = robot.get_C()
print("Coriolis Matrix: \n", C)
G = robot.get_G()
print("Gravity Vector: \n", G)
print("G reg:\n", robot.get_reg_G())
Yr = robot.get_Yr()
Y = robot.get_reg_M() + robot.get_reg_C() + robot.get_reg_G()
print("Regressor: \n", Y[:,[3,4,5,6,7,8,9,     13,14,15,16,17,18,19,   23,24,25,26,27,28,29,   33,34,35,36,37,38,39]])
#print("Regressor pose: \n", Yr)



# ---------Plot-------------------------------------------------------------
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

frames = [np.eye(4)]  # Base frame
for i in range(1, robot.get_numJoints()+1):
    Ti = getattr(robot, f"get_T_0_{i}")()  # Dynamically call robot.get_T_0_i()
    frames.append(Ti)
frames.append(robot.get_T_0_ee())  # End-Effecto frame

def plot_frame(T, ax, name=None, length=0.1):
    origin = T[:3, 3]
    x_axis = T[:3, 0] * length
    y_axis = T[:3, 1] * length
    z_axis = T[:3, 2] * length

    ax.quiver(*origin, *x_axis, color='r', linewidth=2, arrow_length_ratio=0.2)
    ax.quiver(*origin, *y_axis, color='g', linewidth=2, arrow_length_ratio=0.2)
    ax.quiver(*origin, *z_axis, color='b', linewidth=2, arrow_length_ratio=0.2)
    if name:
        ax.text(*origin, name, fontsize=10, color='k')

# Plot all frames
for i, T in enumerate(frames):
    print(f"T_0_{i}:\n", T[:3, 3])
    if i == 0:
        plot_frame(T, ax, 'WORLD', length=0.2)
    elif i == robot.get_numJoints() + 1:
        plot_frame(T, ax, 'EE', length=0.2)
    else:
        plot_frame(T, ax, f'J_{i}', length=0.15)

# Connect origins (optional, to show chain)
origins = np.array([T[:3, 3] for T in frames])
ax.plot(origins[:, 0], origins[:, 1], origins[:, 2], 'k--', linewidth=1)
# --- Add translation labels next to each point ---
for i, p in enumerate(origins):
    label = f"({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f})"
    ax.text(p[0], p[1], p[2] + 0.03, label, color='black', fontsize=9)

# Proper scaling
all_points = np.vstack([T[:3, 3] for T in frames])
max_range = (all_points.max(axis=0) - all_points.min(axis=0)).max() / 2
mid = all_points.mean(axis=0)

for axis, m in zip([ax.set_xlim, ax.set_ylim, ax.set_zlim], mid):
    axis([m - max_range, m + max_range])

# Formatting
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_box_aspect([1,1,1])
ax.view_init(elev=25, azim=45)
ax.set_title('Coordinate Frames')
plt.show()
# ---------------------------------------------------------------------------