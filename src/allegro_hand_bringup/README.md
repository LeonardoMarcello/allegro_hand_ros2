# allegro_hand_bringup

Launch files and configuration to start the Allegro Hand V5 with **ros2_control**.

## Contents

| File | Description |
|---|---|
| `launch/ahand_bringup.py` | Main launch: CAN setup, robot_state_publisher, ros2_control_node, controllers, RViz |
| `launch/rsp.launch.py` | robot_state_publisher only (URDF from xacro) |
| `launch/ahand_isaacsim_bringup.py` | Bringup for Isaac Sim |
| `config/allegro_hand_ros2control.xacro` | URDF + `<ros2_control>` tag (hardware plugin, parameters, joint interfaces and limits) |
| `config/allegro_controllers.yaml` | controller_manager configuration (update rate, controllers, gains) |
| `config/allegro_hand_config.rviz` | RViz configuration |

## Usage

```bash
# real hand (asks for sudo password to set up can0)
ros2 launch allegro_hand_bringup ahand_bringup.py

# fake hardware, no hand connected
ros2 launch allegro_hand_bringup ahand_bringup.py ros2_control_hardware_type:=mock_components

# without RViz
ros2 launch allegro_hand_bringup ahand_bringup.py viz:=false
```

### Launch arguments

| Argument | Default | Description |
|---|---|---|
| `ros2_control_hardware_type` | `physical_device` | `physical_device` → `AllegroHandV5` + `allegro_effort_controller`; `mock_components` → `GenericSystem` + `allegro_position_controller` |
| `viz` | `true` | Start RViz |

With `physical_device`, the launch file reads the CAN `device` from the xacro and runs
`ip link set <device> type can bitrate 1000000` + `up` with sudo.

## Hardware parameters (xacro)

In `config/allegro_hand_ros2control.xacro`, `<hardware>` section:

| Parameter | Example | Description |
|---|---|---|
| `device` | `can0` | CAN interface |
| `handness` | `right` / `left` | Must match the connected hand (checked at startup) |
| `type` | `A` / `B` | Hardware type, must match the connected hand (checked at startup) |
| `position_control` | `false` | `true`: the `position` command interface drives the BHand PD; `false`: the `effort` interface is sent directly |
| `pressure` | `false` | Export the fingertip pressure state interfaces (`<finger>_sensor/pressure`) |
| `kalman_filter_process_noise` / `_measurement_noise` | `10` / `1e-7` | Kalman filter parameters (currently unused) |

Every joint exports the `position` and `effort` command interfaces and the `position`, `velocity`, `effort` state interfaces.

## Controllers (`allegro_controllers.yaml`)

| Controller | Type | Command interface |
|---|---|---|
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | – |
| `allegro_effort_controller` | `allegro_hand_computed_torque/ComputedTorque_Controller` | `effort` |
| `allegro_position_controller` | `joint_trajectory_controller/JointTrajectoryController` | `position` |

`update_rate: 500` Hz. The gains of the computed torque controller are in `allegro_effort_controller.ros__parameters.gains`
(one `kp`/`kd` array of 4 values for fingers, and one for the thumb). See [allegro_hand_ros2_controllers](../allegro_hand_ros2_controllers/README.md).

### Example: position control with the JointTrajectoryController on the real hand

1. Set `<param name="position_control">true</param>` in the xacro.
2. Rebuild `allegro_hand_bringup`, then launch and switch controllers:
```bash
ros2 control switch_controllers --deactivate allegro_effort_controller
ros2 control load_controller --set-state active allegro_position_controller
```
