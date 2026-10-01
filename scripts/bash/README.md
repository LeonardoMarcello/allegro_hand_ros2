# scripts/bash

Command line scripts to save and load Allegro Hand poses.
Both scripts find the workspace from their own location, so they work from any directory. They source
`/opt/ros/humble` and `install/setup.bash` if the workspace is not sourced yet.

Poses are stored in **`workdir/saved_pose.yaml`**:
```yaml
open:
  position: [0.01, 0.12, ..., 0.33]   # 16 values, joint_0_0 ... joint_15_0 [rad]
grasp:
  position: [...]
```

## `save_pose.bash`

Saves the current hand pose with a name, using the `/allegro_hand_service/save_pose` service of the hardware interface.
The other poses in the file are kept, and a pose with the same name is overwritten.

```bash
./scripts/bash/save_pose.bash <pose_name>
```

Example:
```bash
# with the hand in gravity compensation, move the fingers by hand, then:
./scripts/bash/save_pose.bash open
# response: success=True, message='Pose 'open' saved to .../workdir/saved_pose.yaml'
```

Requires: `ahand_bringup.py` running with the real hardware.

## `load_pose.bash`

Reads a pose from `workdir/saved_pose.yaml` and publishes it **once** as `sensor_msgs/JointState` on
`/allegroHand_0/joint_cmd` (computed torque controller command). The controller keeps tracking it until the next command.

```bash
./scripts/bash/load_pose.bash               # list stored poses
./scripts/bash/load_pose.bash <pose_name>   # send the pose
```

Example:
```bash
$ ./scripts/bash/load_pose.bash
Usage: load_pose.bash <pose_name>
Stored poses:
  open
  grasp
$ ./scripts/bash/load_pose.bash grasp
```

- The script waits until a subscriber (the controller) exists. If `allegro_effort_controller` is not active, it waits: stop it with Ctrl+C.
- To publish on another topic, change `TOPIC` at the top of the script.
- It uses the system `/usr/bin/python3` (with PyYAML) to read the file, so it works also with conda active.

> [!CAUTION]
> The hand moves to the pose with a torque step `Kp·(q_r − q)`: load poses near the current configuration.
