#!/bin/bash
set -e

# Source environments
source /opt/ros/humble/setup.bash
source /root/Precision_landing_aruco/install/setup.bash
# Start aruco tracker
ros2 launch ark aruco_tracker.launch.py &
echo "Started aruco_tracker"

echo "Nodes finished. Keeping container alive..."
exec bash
