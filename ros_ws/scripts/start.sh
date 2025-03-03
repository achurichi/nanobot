#!/bin/bash

cd "/home/ros_ws"

if [ "$1" == "sim" ]; then
    # Run Gazebo and Rviz if the script is called with 'sim' as an argument

    # Run Gazebo
    (
        source install/setup.bash
        ros2 launch nanobot_bringup simulation.launch.py world:=./src/nanobot_simulation/worlds/boxes.world
    ) &
    pid1=$!

    # Run rviz
    rviz2 -d ./src/nanobot_simulation/config/nanobot.rviz &
    pid2=$!

    wait $pid1
    wait $pid2
else
    # Run the ros launch file if no arguments are provided

    # Run the FastDDS discovery server
    fastdds discovery --server-id 0 &
    pid1=$!

    # Run the robot launch file
    (
        source install/setup.bash
        ros2 launch nanobot_bringup robot.launch.py
    ) &
    pid2=$!

    wait $pid1
    wait $pid2
fi
