#!/bin/bash

set -e

source /opt/ros/humble/setup.bash

colcon build --symlink-install

source install/setup.bash

ros2 launch eagle_sim dev.launch.py