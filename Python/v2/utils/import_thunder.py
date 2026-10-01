import glob
import os
import sys
import yaml

# IMPORT THUNDER FILES

# 1) Add installation path
rel_paths = glob.glob("/home/leo/Desktop/allegro_hand_ros2_ws/Python/v2/thunder/*/build")

for rel_path in rel_paths:
    if os.path.exists(rel_path):
        sys.path.append(os.path.abspath(rel_path))
    else:
        print(f"Warning: path '{rel_path}' does not exist.")

# 2) Import modules
try:
    from thunder_ahand_thumb_py import thunder_ahand_thumb
    from thunder_ahand_finger_py import thunder_ahand_finger
except ImportError as e:
    print(f"Error: Could not import a thunder module. {e}")

# 3) Defin some utilities
def load_params(robot, config_path):
    with open(config_path, 'r') as f:
        config_robot = yaml.safe_load(f)

    # Dynamic parameters
    robot.set_par_DYN(config_robot['par_DYN'])
    robot.set_par_Dl(config_robot['par_Dl'])
    robot.set_par_REG(config_robot['par_REG'])
    robot.set_par_REG_red(config_robot['par_REG_red'])
    if hasattr(robot,"set_par_Ia"):
        robot.set_par_Ia(config_robot['par_Ia'])
    # Other parameters
    if hasattr(robot,"set_par_Ln2EE"):
        robot.set_par_Ln2EE(config_robot['par_Ln2EE'])
    # Other parameters
    if hasattr(robot,"set_par_world2L0"):
        robot.set_par_world2L0(config_robot['par_world2L0'])
    # Other parameters
    if hasattr(robot,"set_par_gravity"):
        robot.set_par_gravity(config_robot['par_gravity'])