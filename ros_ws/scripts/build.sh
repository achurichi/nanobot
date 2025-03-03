#!/bin/bash

cd "/home/ros_ws"

# Get third party repos
vcs import src < third_parties.repos

# Install udev rules for LDRobot Lidar
cd src/ThirdParty/ldrobot_lidar/scripts
./create_udev_rules.sh

cd "/home/ros_ws"

# Install dependencies
sudo apt-get update
rosdep install --from-paths src --ignore-src -r -y 

# Build ros packages
colcon build --symlink-install
