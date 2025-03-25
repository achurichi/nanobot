#!/bin/bash

# Parse arguments
SIM_MODE="false"

for arg in "$@"; do
  if [ "$arg" == "sim" ]; then
    SIM_MODE="true"
  fi
done

# Get third party repos
cd "/home/nanobot/ros_ws"
vcs import src <third_party.repos

# Install udev rules for LDRobot Lidar
cd src/third_party/ldrobot_lidar/scripts
./create_udev_rules.sh

if [ "$SIM_MODE" == "false" ]; then
  # Install jetgpio library
  cd "../../jetgpio"
  sudo make
  sudo make install
fi

# Install dependencies
cd "/home/nanobot/ros_ws"
sudo apt-get update
rosdep install --from-paths src --ignore-src -r -y

# Build ros packages
if [ "$SIM_MODE" == "true" ]; then
  colcon build --symlink-install --packages-ignore nanobot_imu
else
  colcon build --symlink-install
fi

# Install web app
cd "/home/nanobot/web_app"
npm install

# Build the container for the RealSense camera with GPU
cd "/home/nanobot/ros_ws/src/nanobot_camera/docker"
sudo docker build -f Dockerfile -t realsense_camera .
