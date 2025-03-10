#!/bin/bash

cd "/home/nanobot/ros_ws"

# Get third party repos
vcs import src < third_party.repos

# Install udev rules for LDRobot Lidar
cd src/third_party/ldrobot_lidar/scripts
./create_udev_rules.sh

# Install jetgpio library
cd "../../jetgpio"
sudo make
sudo make install

# Install dependencies
cd "/home/nanobot/ros_ws"
sudo apt-get update
rosdep install --from-paths src --ignore-src -r -y 

# Build ros packages
colcon build --symlink-install
