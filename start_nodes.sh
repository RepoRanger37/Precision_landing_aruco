#!/bin/bash
set -e

# Source environments
source /opt/ros/humble/setup.bash
source /root/microros_ws/install/local_setup.bash
source /root/Precision_landing_aruco/install/setup.bash

# Run nodes in background
ros2 run ark cam &
ros2 run ark fb &
ros2 run ark aruco &
ros2 run precision_land precision_land &
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 921600
# Wait so container stays alive
wait

