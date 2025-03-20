#!/bin/bash

cd "/home/nanobot/ros_ws"

source install/setup.bash

# Parse arguments
SIM_MODE="false"
GENERATE_MAP="false"

for arg in "$@"; do
    if [ "$arg" == "sim" ]; then
        SIM_MODE="true"
    elif [ "$arg" == "generate_map" ]; then
        GENERATE_MAP="true"
    fi
done

# Determine which launch file to run
if [ "$SIM_MODE" == "true" ]; then
    ros2 launch nanobot_bringup simulation.launch.py generate_map:="$GENERATE_MAP"
else
    ros2 launch nanobot_bringup robot.launch.py generate_map:="$GENERATE_MAP"
fi
