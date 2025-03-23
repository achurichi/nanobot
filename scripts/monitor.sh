#!/bin/bash

cd "/home/nanobot/ros_ws"
source install/setup.bash

# Launch monitor
ros2 launch nanobot_bringup monitor.launch.py
# ros2 launch nanobot_bringup monitor.launch.py &
# pid1=$!

# Run the teleop_twist_keyboard node
# TODO: change the default speed
# ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_joy

# wait
