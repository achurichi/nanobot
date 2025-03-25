#!/bin/bash

# Set the environment variables
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/dashing/setup.bash
source /home/nano/ros_ws/install/setup.bash

# Run the ROS 2 launch command in the background
ros2 launch realsense2_camera rs_launch.py config_file:="'/home/nano/camera_config.yaml'" &
ros_pid=$!

# Set up a trap to stop the ROS process when the script exits
trap "kill -SIGINT $ros_pid; wait $ros_pid" EXIT

# Wait for the ROS 2 process to finish
wait $ros_pid
