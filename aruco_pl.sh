#!/bin/bash
set -e

# Source environments
source /opt/ros/humble/setup.bash
source /root/Precision_landing_aruco/install/setup.bash

# Start aruco tracker
ros2 launch ark aruco_tracker.launch.py &
echo "Started aruco_tracker"
sleep 2  # wait 2 seconds

# Start precision_land (runs in foreground)
ros2 run land land

echo "Nodes finished. Keeping container alive..."
exec bash
