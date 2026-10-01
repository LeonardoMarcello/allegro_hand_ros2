# Allegro Hand V5 – ROS 2 Control Workspace

ROS 2 Humble workspace to control the **Allegro Hand V5** (Wonik Robotics) with **ros2_control**.
It adds these parts on top of the original Wonik packages:

- a ros2_control **hardware interface** that talks to the hand over CAN (effort or BHand position control),
- a **computed torque controller** with model-based gravity compensation (from the thunder dynamics model),
- services and scripts to **save and load hand poses**.

> The original Wonik README is kept in [README_wonik.md](README_wonik.md). Personal notes are in [readme.txt](readme.txt).

---

## Packages

| Package | Description | README |
|---|---|---|
| [allegro_hand_bringup](src/allegro_hand_bringup) | Launch files, ros2_control xacro and controller configuration | [README](src/allegro_hand_bringup/README.md) |
| [allegro_hand_hw_interface](src/allegro_hand_hw_interface) | ros2_control `SystemInterface` for the real hand (CAN) and a fake hardware | [README](src/allegro_hand_hw_interface/README.md) |
| [allegro_hand_ros2_controllers](src/allegro_hand_ros2_controllers) | Computed torque controller (gravity compensation + PD) and BHand controller | [README](src/allegro_hand_ros2_controllers/README.md) |
| [allegro_hand_interfaces](src/allegro_hand_interfaces) | Custom services (`SavePose`) | [README](src/allegro_hand_interfaces/README.md) |
| [thunder_ahand](src/thunder_ahand) | Dynamic model of fingers and thumb (thunder library) | [README](src/thunder_ahand/README.md) |
| [allegro_utils](src/allegro_utils) | Utility nodes (wave, trajectory test, GUI, recorder) | [README](src/allegro_utils/README.md) |
| [allegro_hand_description](src/allegro_hand_description) | URDF and meshes (right/left, A/B) | – |
| [allegro_hand_isaacsim](src/allegro_hand_isaacsim) | Isaac Sim real2sim / sim2real bridge | [README](src/allegro_hand_isaacsim/README.md) |
| [allegro_hand_teleoperation](src/allegro_hand_teleoperation) | Teleoperation (MediaPipe hand tracking) | [README](src/allegro_hand_teleoperation/README.md) |
| `allegro_hand_controllers`, `allegro_hand_driver`, `allegro_hand_keyboards`, `allegro_hand_gui`, `allegro_hand_moveit`, `bhand` | Original Wonik packages (standalone `allegro_node`, CAN driver, BHand library, …) | [README_wonik.md](README_wonik.md) |
| [scripts/bash](scripts/bash) | Command line scripts (save/load poses) | [README](scripts/bash/README.md) |

### Architecture

```
                       ┌──────────────────────── ros2_control_node (500 Hz) ────────────────────────┐
 /allegroHand_0/  ───▶ │  allegro_effort_controller      ──effort──▶   AllegroHandV5 (hw interface) │ ──CAN──▶ Allegro Hand
   joint_cmd           │  (computed torque / grav. comp)  ◀─state───   read(): positions, velocity   │ ◀─CAN───
                       │  joint_state_broadcaster ──▶ /joint_states                                   │
                       └──────────────────────────────────────────────────────────────────────────────┘
                              /allegro_hand_service/activate    /allegro_hand_service/save_pose
```

---

## Build

> [!WARNING]
> **Build with conda deactivated.** If a conda environment is active (including `base`), CMake may pick up
> conda libraries (e.g. `libtinyxml2.so.11`) and `ros2_control_node` then crashes when it loads the plugin
> (see [Troubleshooting](#troubleshooting)).

```bash
conda deactivate            # repeat until no environment is active (also leave "base")
cd ~/Desktop/allegro_hand_ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

Build only the ros2_control part:
```bash
colcon build --packages-select allegro_hand_interfaces thunder_ahand allegro_hand_hw_interface allegro_hand_ros2_controllers allegro_hand_bringup
```

Main dependencies (apt): `ros-humble-ros2-control`, `ros-humble-ros2-controllers`, `ros-humble-xacro`, `libyaml-cpp-dev`, `libeigen3-dev`.

---

## Quick start

### 1. Launch the hand (real hardware)
```bash
source install/setup.bash
ros2 launch allegro_hand_bringup ahand_bringup.py
```
The launch file asks for the sudo password to configure `can0` (1 Mbit/s). Then it starts:
- `robot_state_publisher`,
- `ros2_control_node` with the `AllegroHandV5` hardware interface,
- `joint_state_broadcaster`,
- `allegro_effort_controller` (computed torque),
- RViz.

When it starts, the controller runs **gravity compensation**, so you can move the fingers by hand.

### 2. Send a joint command
```bash
ros2 topic pub --once /allegroHand_0/joint_cmd sensor_msgs/msg/JointState \
  "{name: [joint_0_0, joint_1_0, joint_2_0, joint_3_0, joint_4_0, joint_5_0, joint_6_0, joint_7_0,
          joint_8_0, joint_9_0, joint_10_0, joint_11_0, joint_12_0, joint_13_0, joint_14_0, joint_15_0],
   position: [0.0, 0.5, 0.5, 0.5, 0.0, 0.5, 0.5, 0.5, 0.0, 0.5, 0.5, 0.5, 0.8, 0.5, 0.5, 0.5]}"
```
After the first valid command, the controller switches to **computed torque** and tracks the last received command.

### 3. Save and load poses
```bash
./scripts/bash/save_pose.bash open      # store the current pose as "open" in workdir/saved_pose.yaml
./scripts/bash/load_pose.bash           # list the stored poses
./scripts/bash/load_pose.bash open      # send the pose "open" to the controller
```

### 4. Enable / disable the motors
```bash
ros2 service call /allegro_hand_service/activate std_srvs/srv/SetBool "{data: false}"   # zero torque
ros2 service call /allegro_hand_service/activate std_srvs/srv/SetBool "{data: true}"    # enable again
```

### 5. Mock hardware (no hand connected)
```bash
ros2 launch allegro_hand_bringup ahand_bringup.py ros2_control_hardware_type:=mock_components
```
This skips the CAN setup and uses the fake hardware with the `allegro_position_controller` (JointTrajectoryController).

---

## ROS interfaces (summary)

| Name | Type | Description |
|---|---|---|
| `/allegroHand_0/joint_cmd` | `sensor_msgs/JointState` (sub) | Desired joint positions (optional velocities) for the computed torque controller |
| `/joint_states` | `sensor_msgs/JointState` (pub) | Measured joint states (joint_state_broadcaster) |
| `/allegro_effort_controller/command_torque` | `sensor_msgs/JointState` (pub) | Torque sent by the controller (debug) |
| `/allegro_effort_controller/controller_error` | `sensor_msgs/JointState` (pub) | Position error `q_r - q` (debug) |
| `/allegro_effort_controller/set_controller_mode` | `std_srvs/SetBool` | `true`: force gravity compensation, `false`: computed torque |
| `/allegro_hand_service/activate` | `std_srvs/SetBool` | Enable (`true`) / disable (`false`, zero torque) the motors |
| `/allegro_hand_service/save_pose` | `allegro_hand_interfaces/SavePose` | Save the current pose in a YAML file |

Joint order everywhere: `joint_0_0 … joint_3_0` index, `joint_4_0 … joint_7_0` middle, `joint_8_0 … joint_11_0` ring, `joint_12_0 … joint_15_0` thumb.

---

## Safety

- **Zero torque on exit**: on Ctrl+C (hardware `on_shutdown`), on hardware deactivate/error and when the controller is deactivated, zero torque is sent to the hand. The firmware keeps the last torque it received, so this step matters.
- **No jump at startup**: in position mode the hardware holds the measured pose on the first CAN frame. In effort mode the controller starts in gravity compensation.
- Loading a pose far from the current one moves the fingers quickly (P-term step): load poses near the current configuration.

---

## Troubleshooting

**`Failed to load library … liballegro_hand_hw_interface.so … libtinyxml2.so.11: cannot open shared object file`**
The package was built with conda active. Clean and rebuild without conda:
```bash
conda deactivate   # until no env is active
rm -rf build/allegro_hand_hw_interface install/allegro_hand_hw_interface \
       build/allegro_hand_ros2_controllers install/allegro_hand_ros2_controllers
source /opt/ros/humble/setup.bash
colcon build --packages-select allegro_hand_interfaces allegro_hand_hw_interface allegro_hand_ros2_controllers
```
Check the result: `readelf -d install/allegro_hand_hw_interface/lib/liballegro_hand_hw_interface.so | grep tinyxml` must not show `.so.11`.
To stop conda from activating `base` at login: `conda config --set auto_activate_base false`.

**`WRONG HANDEDNESS DETECTED!` / `WRONG TYPE DETECTED!`**
The `handness`/`type` parameters in [allegro_hand_ros2control.xacro](src/allegro_hand_bringup/config/allegro_hand_ros2control.xacro) do not match the connected hand.

**`Cannot initialize CAN device`**
Check that the hand is powered and the USB-CAN adapter is connected (`ip link show can0`).

**The hand does not respond to commands**
Check that the motors are enabled (`/allegro_hand_service/activate` → `true`) and that the controller is active (`ros2 control list_controllers`).
