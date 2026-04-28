#!/bin/bash

cd "/root/ros_ws"
source install/setup.bash
ros2 launch nanobot_bringup monitor.launch.py
