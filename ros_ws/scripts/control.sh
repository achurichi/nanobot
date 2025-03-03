#!/bin/bash

cd "/home/ros_ws"

# Run rviz
rviz2 -d ./src/nanobot_simulation/config/nanobot.rviz &
pid1=$!

# Run the teleop_twist_keyboard node
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_controller/cmd_vel_unstamped

wait $pid1
