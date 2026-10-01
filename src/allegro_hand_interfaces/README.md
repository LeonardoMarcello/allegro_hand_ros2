# allegro_hand_interfaces

Custom ROS 2 interfaces for the Allegro Hand packages.

## Services

### `SavePose.srv`

```
string name        # pose name, e.g. "open"
string file_path   # YAML file where the pose is stored
---
bool success
string message
```

Provided by the hardware interface as `/allegro_hand_service/save_pose`
(see [allegro_hand_hw_interface](../allegro_hand_hw_interface/README.md#services)).

```bash
ros2 interface show allegro_hand_interfaces/srv/SavePose

ros2 service call /allegro_hand_service/save_pose allegro_hand_interfaces/srv/SavePose \
  "{name: 'open', file_path: '/home/leo/Desktop/allegro_hand_ros2_ws/workdir/saved_pose.yaml'}"
```

## Use in C++

```cmake
find_package(allegro_hand_interfaces REQUIRED)
ament_target_dependencies(my_target allegro_hand_interfaces)
```
```cpp
#include "allegro_hand_interfaces/srv/save_pose.hpp"
```
