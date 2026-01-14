#!/bin/bash
set -e

# Source ROS 2 setup
source /opt/ros/${ROS_DISTRO}/setup.bash

# Source workspace setup if it exists
if [ -f /ros_ws/install/setup.bash ]; then
    source /ros_ws/install/setup.bash
fi

# Execute command
exec "$@"
