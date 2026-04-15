#!/bin/bash

# Get third party repos
cd "/home/nanobot/ros_ws"

colcon build --symlink-install --packages-up-to ignition-math6
source install/setup.bash && colcon build --symlink-install --packages-up-to nanobot_bringup
