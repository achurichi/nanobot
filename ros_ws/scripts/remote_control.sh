#!/bin/bash

cd "/home/ros_ws"

if [ $# -ne 1 ]; then
    echo "Usage: $(basename "$0") <new_ip_address>"
    exit 1
fi

NEW_IP="$1"
CONFIG_FILE="/home/ros_ws/super_client_configuration_file.xml"

if [ ! -f "$CONFIG_FILE" ]; then
    echo "Error: File $CONFIG_FILE not found!"
    exit 1
fi

export ROS_DISCOVERY_SERVER="$NEW_IP:11811"

# Run rviz
rviz2 -d ./src/nanobot_simulation/config/nanobot.rviz &
pid1=$!

# Run the teleop_twist_keyboard node
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_controller/cmd_vel_unstamped

wait $pid1
