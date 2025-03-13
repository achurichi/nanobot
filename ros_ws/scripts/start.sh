#!/bin/bash

cd "/home/nanobot/ros_ws"

if [ "$1" == "sim" ]; then
    # Run Gazebo if the script is called with 'sim' as an argument
    source install/setup.bash
    ros2 launch nanobot_bringup simulation.launch.py
else
    # Run the robot launch file if no arguments are provided
    source install/setup.bash
    ros2 launch nanobot_bringup robot.launch.py
fi
