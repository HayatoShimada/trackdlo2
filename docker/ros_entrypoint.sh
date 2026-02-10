#!/bin/bash
set -e

# Source ROS2 base setup
source /opt/ros/${ROS_DISTRO}/setup.bash

# Source workspace setup if it exists
if [ -f "/ros2_ws/install/setup.bash" ]; then
    source /ros2_ws/install/setup.bash
fi

exec "$@"
