#!/bin/bash

cd "/home/nanobot/ros_ws"

# Run rviz
rviz2 -d ./src/nanobot_simulation/config/nanobot.rviz &
pid1=$!

# Run rqt-image-view
# TODO: auto select the camera topic
ros2 run rqt_image_view rqt_image_view & 
pid2=$!

# Run the teleop_twist_keyboard node
# TODO: change the default speed
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_controller/cmd_vel_unstamped

wait $pid1
wait $pid2