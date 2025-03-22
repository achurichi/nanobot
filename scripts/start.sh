#!/bin/bash

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

# Start the web app
cd "/home/nanobot/web_app"
VITE_CONNECTION_URL=ws://$(hostname -I | awk '{print $1}'):9090 \
VITE_CMD_VEL_TOPIC=/cmd_vel_joy \
VITE_CAMERA_TOPIC=/camera/realsense_camera/color/image_raw/compressed \
npm run dev &
pid1=$!

# ROS 2 launch
cd "/home/nanobot/ros_ws"
source install/setup.bash
if [ "$SIM_MODE" == "true" ]; then
    ros2 launch nanobot_bringup simulation.launch.py generate_map:="$GENERATE_MAP"
else
    ros2 launch nanobot_bringup robot.launch.py generate_map:="$GENERATE_MAP"
fi

wait