import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider


import utils.import_thunder as thunder

index = thunder.thunder_ahand_finger()
middle = thunder.thunder_ahand_finger()
ring = thunder.thunder_ahand_finger()
thumb = thunder.thunder_ahand_thumb()

# Load config

ring.set_par_world2L0( [0, -0.0450, -0.0020, 0, 0, 0.0872665])
index.set_par_world2L0([0, 0.0432, -0.0022, 0, 0, -0.0872665])
middle.set_par_world2L0([0, -0.0009, 0, 0, 0, 0])
fingers = [index, middle, ring, thumb]
thumb.set_q([0,0.3,0.5,0.5])

# ---------Plot-------------------------------------------------------------
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

frames = {"WORLD": np.eye(4)}  # Base frame
for f in range(len(fingers)):
    for i in range(1, fingers[f].numJoints+1):
        Ti = getattr(fingers[f], f"get_T_w_{i}")()  # Dynamically call robot.get_T_0_i()
        frames[f"O{f+1}_{i}"] =Ti
    frames[f"O{f+1}_EE"] = fingers[f].get_T_w_ee()  # End-Effector frame

def plot_frame(T, ax, name=None, length=0.1):
    origin = T[:3, 3]
    x_axis = T[:3, 0] * length
    y_axis = T[:3, 1] * length
    z_axis = T[:3, 2] * length

    ax.quiver(*origin, *x_axis, color='r', linewidth=2, arrow_length_ratio=.2)
    ax.quiver(*origin, *y_axis, color='g', linewidth=2, arrow_length_ratio=.2)
    ax.quiver(*origin, *z_axis, color='b', linewidth=2, arrow_length_ratio=.2)
    if name:
        ax.text(*origin, name, fontsize=10, color='k')

# Plot all frames
for key, T in frames.items():
    plot_frame(T, ax, key, length=0.015)

# Connect origins (optional, to show chain)
origins = np.array([T[:3, 3] for T in frames.values()])
ax.plot(origins[:, 0], origins[:, 1], origins[:, 2], 'k--', linewidth=1)
# --- Add translation labels next to each point ---
for i, p in enumerate(origins):
    label = f"({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f})"
    ax.text(p[0], p[1], p[2] + 0.03, label, color='black', fontsize=9)

# Proper scaling
all_points = np.vstack([T[:3, 3] for T in frames.values()])
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