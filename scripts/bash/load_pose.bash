#!/usr/bin/env bash
# Publish a pose stored in workdir/saved_pose.yaml as joint command for the computed torque controller
# Usage: load_pose.bash <pose_name>    (without argument: list the stored poses)
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
POSE_FILE="$WS_DIR/workdir/saved_pose.yaml"
TOPIC="/allegroHand_0/joint_cmd"
PYTHON=/usr/bin/python3     # system python (has PyYAML), independent of conda

if [ ! -f "$POSE_FILE" ]; then
    echo "No saved poses: $POSE_FILE not found"
    exit 1
fi

if [ $# -ne 1 ]; then
    echo "Usage: $(basename "$0") <pose_name>"
    echo "Stored poses:"
    $PYTHON -c 'import sys, yaml; [print("  " + k) for k in (yaml.safe_load(open(sys.argv[1])) or {})]' "$POSE_FILE"
    exit 1
fi

# build the JointState message from the stored pose
MSG=$($PYTHON - "$POSE_FILE" "$1" <<'PY'
import sys, yaml
poses = yaml.safe_load(open(sys.argv[1])) or {}
name = sys.argv[2]
if name not in poses:
    sys.exit(f"Pose '{name}' not found. Stored poses: {', '.join(poses)}")
position = [float(q) for q in poses[name]['position']]
if len(position) != 16:
    sys.exit(f"Pose '{name}' has {len(position)} joints, expected 16")
names = [f"joint_{i}_0" for i in range(16)]
print(f"{{name: {names}, position: {position}}}")
PY
)

# source ROS 2 and the workspace if not already done
if [[ ":$AMENT_PREFIX_PATH:" != *":$WS_DIR/install/"* ]]; then
    source /opt/ros/humble/setup.bash
    source "$WS_DIR/install/setup.bash"
fi

# publish once, waiting until the controller is subscribed
ros2 topic pub --once -w 1 "$TOPIC" sensor_msgs/msg/JointState "$MSG"
