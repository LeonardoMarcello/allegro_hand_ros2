import numpy as np
import matplotlib.pyplot as plt
import sys
import os

# Load the fingers model (Pinocchio Ahand)
# ---------------------------------------------------------------------------
from AHAND import AHAND, MESH_DIR, URDF_PATH, XML_PATH # Mujoco AHAND class
mjHand = AHAND(URDF_PATH, MESH_DIR) # Mujoco AHAND instance



# Load the fingers model (Thunder Ahand)
# ---------------------------------------------------------------------------
build_paths = ["./ahand_finger_generatedFiles/build", "./ahand_thumb_generatedFiles/build"]
# Note: This is not needed if the .so file is in the same directory as the python script
for build_path in build_paths:
    so_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), build_path)
    if os.path.exists(so_path):
        sys.path.append(os.path.abspath(so_path))
    else:
        print(f"Warning: path '{so_path}' does not exist.")

from thunder_ahand_finger_py import thunder_ahand_finger
from thunder_ahand_thumb_py import thunder_ahand_thumb
ring = thunder_ahand_finger()
ring.load_conf(os.path.join(os.path.dirname(os.path.abspath(__file__)),"./config/thunder/ahand_ring_conf.yaml"))
index = thunder_ahand_finger()
index.load_conf(os.path.join(os.path.dirname(os.path.abspath(__file__)),"./config/thunder/ahand_index_conf.yaml"))
middle = thunder_ahand_finger()
middle.load_conf(os.path.join(os.path.dirname(os.path.abspath(__file__)),"./config/thunder/ahand_middle_conf.yaml"))
thumb = thunder_ahand_thumb()
thumb.load_conf(os.path.join(os.path.dirname(os.path.abspath(__file__)),"./config/thunder/ahand_thumb_conf.yaml"))
fingers = [index, middle, ring, thumb]


# Fingertip base location
# ---------------------------------------------------------------------------
mj_finger_base_indeces = {
                # 'index' :   mjHand.model.getFrameId('joint_0_0'), # red
                # 'middle':   mjHand.model.getFrameId('joint_4_0'), # green
                # 'ring'  :   mjHand.model.getFrameId('joint_8_0'), # blue
                # 'thumb' :   mjHand.model.getFrameId('joint_12_0') # cyan
                'index' :   mjHand.model.getFrameId('joint_1_0'), # red
                'middle':   mjHand.model.getFrameId('joint_5_0'), # green
                'ring'  :   mjHand.model.getFrameId('joint_9_0'), # blue
                'thumb' :   mjHand.model.getFrameId('joint_13_0') # cyan
        }
print(mj_finger_base_indeces)
print(mjHand.end_effector_indeces)

mjHand.updateConfig()
x_i = mjHand.data.oMf[mj_finger_base_indeces['index']].translation.copy()
x_ii = mjHand.data.oMf[mj_finger_base_indeces['middle']].translation.copy()
x_iii = mjHand.data.oMf[mj_finger_base_indeces['ring']].translation.copy()
x_iv = mjHand.data.oMf[mj_finger_base_indeces['thumb']].translation.copy()
x = np.hstack([x_i, x_ii, x_iii, x_iv])
print(x_i*1000)
print(x_ii*1000)
print(x_iii*1000)
print(x_iv*1000)
print("")

y_i = fingers[0].get_T_0_1()[0:3,3]
y_ii = fingers[1].get_T_0_1()[0:3,3]
y_iii = fingers[2].get_T_0_1()[0:3,3]
y_iv = fingers[3].get_T_0_1()[0:3,3]
y = np.hstack([y_i, y_ii, y_iii, y_iv])
print(y_i*1000)
print(y_ii*1000)
print(y_iii*1000)
print(y_iv*1000)
print("")

print("Fingertip base position error [mm]:")
print((x[:3] - y[:3])*1000)
print((x[3:6] - y[3:6])*1000)
print((x[6:9] - y[6:9])*1000)
print((x[9:12] - y[9:12])*1000)
print("")

# Forward Kinematic on random config
# ---------------------------------------------------------------------------
#middle.set_world2L0(np.array([0, 0, 0]))
#index.set_world2L0(np.array([0, 0.44, 0]))
#ring.set_world2L0(np.array([0,-0.44, 0]))
#thumb.set_world2L0(np.array([0,-0.44, -0.44]))
mj_set_func = [mjHand.set_index, mjHand.set_middle, mjHand.set_ring, mjHand.set_thumb]
for i in range(len(fingers)):
    q = np.random.rand(fingers[i].get_numJoints())
    fingers[i].set_q(q)
    mj_set_func[i](q)
mjHand.updateConfig()


mj_ee = [mjHand.computeForwardKinematic()[3*i:3*i+3] for i in range(len(fingers))]
for f in range(len(fingers)):
    print(f"position of finger {f+1} at [m]:")
    print(fingers[f].get_T_0_ee()[0:3,3])
    print(mj_ee[f])
    print(f"Error of finger {f+1} [mm]:")
    print((fingers[f].get_T_0_ee()[0:3,3] - mj_ee[f])*1000)
    print("")


mjHand.initGepettoViewer()
mjHand.show()

# ---------Plot-------------------------------------------------------------
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

frames = {"WORLD": np.eye(4)}  # Base frame
for f in range(len(fingers)):
    for i in range(1, fingers[f].get_numJoints()+1):
        Ti = getattr(fingers[f], f"get_T_0_{i}")()  # Dynamically call robot.get_T_0_i()
        frames[f"O{f+1}_{i}"] =Ti
    frames[f"O{f+1}_EE"] = fingers[f].get_T_0_ee()  # End-Effector frame

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