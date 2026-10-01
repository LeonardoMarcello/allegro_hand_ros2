# thunder_ahand

Dynamic model of the Allegro Hand V5 fingers, generated with the **thunder** dynamics library.
It provides two C++ classes, used by the [computed torque controller](../allegro_hand_ros2_controllers/README.md):

| Class | Header | Description |
|---|---|---|
| `thunder_ahand_finger` | `thunder_ahand/thunder_ahand_finger.h` | 4-DoF finger (index, middle, ring) |
| `thunder_ahand_thumb` | `thunder_ahand/thunder_ahand_thumb.h` | 4-DoF thumb |

## Usage

```cpp
#include "thunder_ahand/thunder_ahand_finger.h"

thunder_ahand_finger index;
index.load_par("<path>/ahand_index_identification_par.yaml");   // dynamic parameters

index.set_q(q);       // Eigen::Vector<double,4> joint positions
index.set_dq(dq);     // joint velocities
index.set_dqr(dq_r);  // reference velocities
index.set_ddqr(ddq_r);

Eigen::Matrix<double,4,1> G = index.get_G();   // gravity torque
Eigen::Matrix<double,4,4> C = index.get_C();   // Coriolis matrix
```

Gravity direction and base pose can be set with `set_par_gravity(...)` and `set_par_world2L0(...)`.

CMake:
```cmake
find_package(thunder_ahand REQUIRED)
target_link_libraries(my_target thunder_ahand::thunder_ahand)
```

## Configuration

| File | Description |
|---|---|
| `config/ahand_finger_conf.yaml`, `config/ahand_thumb_conf.yaml` | Kinematic/dynamic configuration used for generation |
| `config/ahand_finger_par.yaml`, `config/ahand_thumb_par.yaml` | Nominal parameters |
| `config/updated_wonik_urdf/` | Parameters from the updated Wonik URDF |

The identified parameters used on the real hand are in
[allegro_hand_ros2_controllers/config/thunder](../allegro_hand_ros2_controllers/config/thunder).
