# allegro_utils

Utility nodes and tests for the Allegro Hand.

## Executables

| Executable | Description |
|---|---|
| `wave.py` | Publishes a finger wave motion (parameter `output_topic`, default `/allegroHand_0/joint_cmd`) |
| `test_traject.py` | Publishes test trajectories (parameter `output_topic`) |
| `gui.py` | GUI to command the hand (parameters `input_topic`, `output_topic`, `control_topic`) |
| `moveit_joint_broadcaster.py` | Forwards MoveIt joint states to a command topic (parameters `input_topic`, `output_topic`) |
| `my_recorder.py` | Records data (for identification / plots) |
| `test_thunder_v2` | C++ test of the thunder dynamic model |

## Examples

```bash
ros2 run allegro_utils wave.py --ros-args -p output_topic:=/allegroHand_0/joint_cmd
ros2 run allegro_utils moveit_joint_broadcaster.py
```

## Launch

```bash
ros2 launch allegro_utils ahand_test.py
```

## Other folders

| Folder | Description |
|---|---|
| `config/thunder/` | thunder model parameters used by the tests |
| `module/`, `allegro_utils/` | Python modules |
| `scripts/description/` | URDF used by the scripts |
