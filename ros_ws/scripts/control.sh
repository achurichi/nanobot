#!/bin/bash

cd "/home/nanobot/ros_ws"

source install/setup.bash

# Run control launch file
ros2 launch nanobot_bringup control.launch.py & 
pid1=$!

# Run the teleop_twist_keyboard node
# TODO: change the default speed
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_controller/cmd_vel_unstamped

wait $pid1