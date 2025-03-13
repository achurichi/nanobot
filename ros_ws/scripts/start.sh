#!/bin/bash

cd "/home/nanobot/ros_ws"

source install/setup.bash

if [ "$1" == "sim" ]; then
    # Run Gazebo if the script is called with 'sim' as an argument
    ros2 launch nanobot_bringup simulation.launch.py
else
    # Run the robot launch file if no arguments are provided
    ros2 launch nanobot_bringup robot.launch.py
fi
