git link:
https://github.com/Wonikrobotics-git/allegro_hand_ros2_v5.git


# Models Version(Num of Finger)  Hedness  Geartype:
# Allegro Hand V5(4F) _Right_|Left A|_B_

# Leo Mod:
#----------
# Launcher:
#   - allegro_node_sim.launch.py  -> Set up interfaces with isaacsim simulator
#
# Isaacsim interfaces:
#   - sim.cpp   -> (node allegro_hand_sim) map moveit states to issac commanded joint
#
# moveit:
#   - demo_isaac.py -> launcher for moveit in isaacsim
#
# Controllers:
# 1) allegro_node_sim       ->  Simulation controller to disable can check, Allowing BHand library usage.
#                               Desired joint states are directly applied to a dummy Allegro
# 2) allegro_node_torque    ->  torque controller to manage torque command.
#                               Subscribe to /torque_cmd topic and send command to the Allegro Hand
#                               _TO DO_: saturation, check units (it should be mA)
#
#
# AllegroHandDrv.cpp -> aggiunto messaggio per pressure sensor
# remark: topic di tipo array con in ordine indice, medio, anulare, pollice con
# pressure in (Pa) units
# Oss: credo nuovo firmware non ne abbia bisogno, hanno cambiat ID


# -----------------------------------------------------------------------------------------------------
# If conda environment is active `ros2 run allegro_hand_gui allegro_hand_gui_node' may not work.
# Link the system lib GLIBCXX_3.4.30 in the conda environment by typing:
sudo ln -sf /usr/lib/x86_64-linux-gnu/libstdc++.so.6 ${CONDA_PREFIX}/lib/libstdc++.so.6



# Moveit: by using launch
ros2 launch allegro_hand_controllers allegro_hand.launch.py HAND:=right TYPE:=B MOVEIT:=true
# it will publish on robot_description the state of a dummy allegro, while joints state are published on
# topic /joint_states. Up to now, use /joint_states as command topic of articulated controller of isaacsim
ros2 run allegro_utils moveit_joint_broadcaster



# Plot: Plotjuggler
sudo apt install ros-$ROS_DISTRO-plotjuggler-ros
ros2 run plotjuggler plotjuggler -n     # (-n start with no memes)

# UTILS:
ros2 run allegro_utils wave --ros-args -p speed:=1.5    # publish finger wave with speed rad/s
ros2 run allegro_hand_keyboards allegro_hand_keyboard   # use built-in keyboard interface

# for running using pinocchio conda environments. type the following in a terminal
conda activate pinocchio                                                    # activate conda environment
export PYTHONPATH=$CONDA_PREFIX/lib/python3.10/site-packages:$PYTHONPATH    # run ros2 using conda environment
export LD_LIBRARY_PATH=$CONDA_PREFIX/lib:$LD_LIBRARY_PATH                   # use newer dynamic linker needed from pinocchio binary (GLIBCXX_3.4.31)





# REINFORCEMENT LEARNING
cd isaaclab
./isaaclab.sh -p scripts/reinforcement_learning/skrl/play.py --task=Isaac-Repose-Cube-Allegro-Direct-v0 --checkpoint=logs/skrl/allegro_hand/2025-09-13_17-29-41_ppo_torch/checkpoints/best_agent.pt --num_env=1
