#!/bin/bash

# Launch camera
gnome-terminal -- bash -c "source /opt/ros/foxy/setup.bash; source ~/ros2_ws/install/setup.bash; ros2 launch camera_pkg camera_launch.py; exec bash"

# Launch midas
gnome-terminal -- bash -c "source /opt/ros/foxy/setup.bash; source ~/ros2_ws/install/setup.bash; ros2 run depth_pkg midas_node; exec bash"
