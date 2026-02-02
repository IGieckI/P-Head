#!/bin/bash
set -e

echo "--- Building Workspace ---"
colcon build --symlink-install

echo "--- Sourcing Workspace ---"
source install/setup.bash

echo "--- Launching Robot System ---"

# Check if exactly 2 arguments (IPs hopefully) are passed
if [ "$#" -eq 2 ]; then
    echo "Using provided IPs: Cam1=$1, Cam2=$2"
    ros2 launch master robot_system.launch.py ip_cam1:="$1" ip_cam2:="$2"
else
    ros2 launch master robot_system.launch.py "$@"
fi
