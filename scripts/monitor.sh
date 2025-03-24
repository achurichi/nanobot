#!/bin/bash

# Parse arguments
USE_SIM_TIME="false"

for arg in "$@"; do
  if [ "$arg" == "sim" ]; then
    USE_SIM_TIME="true"
  fi
done

# Launch monitor
cd "/home/nanobot/ros_ws"
source install/setup.bash
ros2 launch nanobot_bringup monitor.launch.py use_sim_time:="$USE_SIM_TIME"
