#!/bin/bash
set -e

# Source environments
source /opt/ros/humble/setup.bash
source /root/microros_ws/install/local_setup.bash
source /root/Precision_landing_aruco/install/setup.bash

# Start micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 921600 &
echo "Started micro_ros_agent"
sleep 3  # wait 3 seconds for initialization

# Start aruco tracker
ros2 launch ark aruco_tracker.launch.py &
echo "Started aruco_tracker"
sleep 2  # wait 2 seconds

# Start precision_land (runs in foreground)
ros2 run precision_land precision_land

echo "Nodes finished. Keeping container alive..."
exec bash

