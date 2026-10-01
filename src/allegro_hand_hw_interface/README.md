# allegro_hand_hw_interface

ros2_control hardware interfaces (`hardware_interface::SystemInterface`) for the Allegro Hand V5.

| Plugin | Description |
|---|---|
| `allegro_hand_hw_interface/AllegroHandV5` | Real hand over SocketCAN (Wonik `AllegroHandDrv` driver + BHand library) |
| `allegro_hand_hw_interface/GenericSystem` | Fake hardware (commands copied to states), used with `mock_components` |

## How `AllegroHandV5` works

```
read():   read CAN frames ──▶ all 4 fingers received? ──yes──▶ update position / velocity, new_data_ = true
write():  new_data_? ──no──▶ nothing (the hand keeps the last torque)
                     └─yes─▶ [position mode: BHand PD computes torque]
                             ──▶ active_? else zero ──▶ send torque ──▶ resetJointInfoReady()
```

- The hand sends positions every **3 ms**. Torque is computed and sent **only when a complete new frame arrives**, as in the
  original Wonik `allegro_node`. BHand never runs on old positions, and the CAN bus does not get extra frames.
- Velocity is the finite difference over the time elapsed since the last complete frame.
- On the first valid frame after each activation, the interface:
  - checks handedness and type (`handness`, `type` parameters); on a mismatch it returns an error and closes the driver,
  - in position mode, sets `desired_position` to the measured pose (the hand holds its pose, no jump to 0).

### Control modes (fixed by the `position_control` parameter)

| `position_control` | Command interface used | Torque sent to the hand |
|---|---|---|
| `false` (default) | `effort` | the effort command, as received |
| `true` | `position` | BHand `JOINT_PD` computed from `position` (BHand created from `handness`/`type`) |

The state filtering also depends on the mode: raw positions in position mode, and a low-pass filter on position and velocity in effort mode.

### Exported interfaces

- State: `joint_X_0/position`, `joint_X_0/velocity`, `joint_X_0/effort` (last torque sent), plus `<finger>_sensor/pressure` if `pressure: true`
- Command: `joint_X_0/position`, `joint_X_0/effort`

## Services

The interface starts its own node, **`allegro_hand_service`**, with these services:

| Service | Type | Description |
|---|---|---|
| `/allegro_hand_service/activate` | `std_srvs/srv/SetBool` | `false`: send zero torque (motors off), `true`: enable again |
| `/allegro_hand_service/save_pose` | `allegro_hand_interfaces/srv/SavePose` | Add or overwrite the current pose `name` in the YAML file `file_path` |

```bash
# disable / enable the motors
ros2 service call /allegro_hand_service/activate std_srvs/srv/SetBool "{data: false}"
ros2 service call /allegro_hand_service/activate std_srvs/srv/SetBool "{data: true}"

# save the current pose (or use scripts/bash/save_pose.bash)
ros2 service call /allegro_hand_service/save_pose allegro_hand_interfaces/srv/SavePose \
  "{name: 'open', file_path: '$HOME/Desktop/allegro_hand_ros2_ws/workdir/saved_pose.yaml'}"
```

File format (other poses in the file are kept):
```yaml
open:
  position: [0.01, 0.12, ..., 0.33]   # 16 values, joint_0_0 ... joint_15_0 [rad]
```
The service fails (`success: false`) if the name or path is empty, if the hand has not sent any position yet, or if
the file cannot be read or written.

## Safety / lifecycle

| Transition | Action |
|---|---|
| `on_configure` | open CAN, create the `allegro_hand_service` node |
| `on_activate` | enable the motors, re-check the hand, hold the current pose on the next frame |
| `on_deactivate` | **zero torque**, stop the service node |
| `on_shutdown` (Ctrl+C) | **zero torque**, stop the service node |
| `on_error` / destructor | **zero torque** |

The zero torque is sent after a complete CAN frame arrives (waits at most ~20 ms), because the driver does not send torque otherwise.

> Note: after `on_deactivate` the `allegro_hand_service` node is not created again on the next `on_activate`.

## Parameters

See [allegro_hand_bringup](../allegro_hand_bringup/README.md#hardware-parameters-xacro).

## Files

| File | Description |
|---|---|
| `src/allegro_hand_interface.cpp` | `AllegroHandV5` plugin |
| `src/generic_system.cpp` | `GenericSystem` fake hardware |
| `src/AllegroHandDrv.cpp`, `src/candrv/` | Wonik CAN driver (vendor code, not modified) |
| `src/kalman_filter_joint.cpp` | Joint Kalman filter (currently unused) |
| `allegro_hand_interface.xml` | pluginlib description |
