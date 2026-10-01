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

# ------------------------------------------------------------------------------------------------------------------
# Specific transformation for AllegroHand DH to URDF frames
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
# ------------------------------------------------------------------------------------------------------------------

import xml.etree.ElementTree as ET
tree = ET.parse("/home/leo/Desktop/allegro_hand_ros2_ws/Python/v2/assets/allegro_hand_v5/urdf/allegro_hand_v5_leo_tmp.xml") # <-- template xml 
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
tree.write("/home/leo/Desktop/allegro_hand_ros2_ws/Python/v2/assets/allegro_hand_v5/urdf/allegro_hand_v5_leo_tmp.xml", encoding="utf-8", xml_declaration=True) # <-- modified xml
print("xml Exported")