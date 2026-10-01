# allegro_hand_ros2_controllers

ros2_control controllers for the Allegro Hand V5.

| Plugin | Description |
|---|---|
| `allegro_hand_computed_torque/ComputedTorque_Controller` | Gravity compensation + PD with a model-based gravity term (thunder dynamics) |
| `allegro_hand_bhand_controller/BHand_Controller` | Torque from the Wonik BHand library (JOINT_PD) |

## Computed torque controller

Control law, for each finger `f` (index, middle, ring, thumb):

```
tau_f = G_f(q) + Kp_f * (q_r - q) + Kd_f * (dq_r - dq)
```

- `G_f(q)`: gravity torque from the thunder model ([thunder_ahand](../thunder_ahand/README.md)). The identified parameters are loaded from
  `config/thunder/ahand_<finger>_identification_par.yaml`.
- `Kp_f`, `Kd_f`: diagonal gains, 4 values per finger (the same for index/middle/ring, separate for the thumb).
- `q_r`, `dq_r`: last received command (`dq_r = 0` if the message has no 16 velocities).

### Modes

| Condition | Output |
|---|---|
| no valid command received since activation | **gravity compensation**, `tau = G(q)` |
| a valid command received | **computed torque**, `tau` as above, tracks the last command |
| `set_controller_mode` → `true` | gravity compensation, even with a command |
| controller deactivated | **zero torque** on all joints |

The controller does not go back to gravity compensation when the planner stops publishing: it holds the last command.
A message without 16 positions is ignored (with a throttled error), and the last valid command is kept.

### Topics and services

| Name | Type | Description |
|---|---|---|
| `/allegroHand_0/joint_cmd` | `sensor_msgs/JointState` (sub) | Command: 16 `position` (+ optional 16 `velocity`) in joint order |
| `~/command_torque` | `sensor_msgs/JointState` (pub) | Torque sent (`effort`) and measured position (`position`) |
| `~/controller_error` | `sensor_msgs/JointState` (pub) | Position error `q_r - q` |
| `~/pbhand_command_torque` | `sensor_msgs/JointState` (pub) | Torque the BHand library would compute (comparison only, not sent) |
| `~/set_controller_mode` | `std_srvs/SetBool` | `true`: gravity compensation, `false`: computed torque |

`~` is the controller name, for example `/allegro_effort_controller/…`.

### Parameters

```yaml
allegro_effort_controller:
  ros__parameters:
    gains:
      finger:
        kp: [0.5, 1.0, 0.95, 0.5]     # [Nm/rad] joints 0..3 of index/middle/ring
        kd: [0.05, 0.06, 0.045, 0.02]
      thumb:
        kp: [1.0, 0.7, 0.35, 0.2]
        kd: [0.06, 0.02, 0.03, 0.03]
    BestEffort_QOS: false             # optional: best-effort QoS for command subscriber / publishers
```

### Examples

```bash
# check the controller state
ros2 control list_controllers

# send a command
ros2 topic pub --once /allegroHand_0/joint_cmd sensor_msgs/msg/JointState \
  "{name: [joint_0_0, joint_1_0, joint_2_0, joint_3_0, joint_4_0, joint_5_0, joint_6_0, joint_7_0,
          joint_8_0, joint_9_0, joint_10_0, joint_11_0, joint_12_0, joint_13_0, joint_14_0, joint_15_0],
   position: [0.0, 0.5, 0.5, 0.5, 0.0, 0.5, 0.5, 0.5, 0.0, 0.5, 0.5, 0.5, 0.8, 0.5, 0.5, 0.5]}"

# force gravity compensation / back to computed torque
ros2 service call /allegro_effort_controller/set_controller_mode std_srvs/srv/SetBool "{data: true}"
ros2 service call /allegro_effort_controller/set_controller_mode std_srvs/srv/SetBool "{data: false}"

# plot the error
ros2 topic echo /allegro_effort_controller/controller_error
```

### Notes

- The first command after gravity compensation produces a torque step `Kp·e`. Start the planner from the measured pose
  (the JointTrajectoryController does this).
- The command is a step signal between messages: publish at a high rate or interpolate for smooth motion.
- The BHand instance inside this controller is fixed to right hand B type and is used only for the `~/pbhand_command_torque` topic.

## Files

| File | Description |
|---|---|
| `src/allegro_hand_computed_torque.cpp` | Computed torque controller |
| `src/allegro_hand_bhand_controller.cpp` | BHand controller |
| `config/ahand_controllers.yaml` | Parameter template (the running configuration is in `allegro_hand_bringup`) |
| `config/thunder/` | Identified dynamic parameters of each finger |
| `ahand_controllers_plugin.xml` | pluginlib description |
