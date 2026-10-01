#!/usr/bin/env bash
# Save the current Allegro Hand pose in workdir/saved_pose.yaml
# Usage: save_pose.bash <pose_name>
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
POSE_FILE="$WS_DIR/workdir/saved_pose.yaml"
SERVICE="/allegro_hand_service/save_pose"

if [ $# -ne 1 ]; then
    echo "Usage: $(basename "$0") <pose_name>"
    exit 1
fi

# source ROS 2 and the workspace if not already done (needed for allegro_hand_interfaces)
if [[ ":$AMENT_PREFIX_PATH:" != *":$WS_DIR/install/"* ]]; then
    source /opt/ros/humble/setup.bash
    source "$WS_DIR/install/setup.bash"
fi

ros2 service call "$SERVICE" allegro_hand_interfaces/srv/SavePose \
    "{name: '$1', file_path: '$POSE_FILE'}"
