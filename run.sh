#!/usr/bin/env bash

set -e

cd /home/eagle/ws

source /opt/ros/humble/setup.bash

echo "BUILDANDO tutto"
colcon build --symlink-install

source install/setup.bash

echo "Startando..."
ros2 launch eagle_control control.launch.py