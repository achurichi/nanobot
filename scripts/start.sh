#!/bin/bash

# Parse arguments
SIM_MODE="false"
GENERATE_MAP="false"
NO_GPU="false"

for arg in "$@"; do
  if [ "$arg" == "sim" ]; then
    SIM_MODE="true"
  elif [ "$arg" == "generate_map" ]; then
    GENERATE_MAP="true"
  elif [ "$arg" == "no_gpu" ]; then
    NO_GPU="true"
  fi
done

# Start the web app
cd "/home/nanobot/web_app"
VITE_CONNECTION_URL=ws://$(hostname -I | awk '{print $1}'):9090 \
VITE_CMD_VEL_TOPIC=/cmd_vel_joy \
VITE_CAMERA_TOPIC=/camera/color/image_raw/compressed \
  npm run dev &
pid1=$!

# If not in simulation mode and using GPU, start the RealSense docker container
if [[ "$SIM_MODE" == "false" && "$NO_GPU" == "false" ]]; then
  # TODO: show container logs
  container_id=$(sudo docker run \
    --privileged \
    --user 'nano' \
    --network host \
    --pid host \
    --ipc host \
    --runtime nvidia \
    --rm \
    --detach \
    realsense_camera)
  echo "RealSense Camera container started"
fi

# ROS 2 launch
cd "/home/nanobot/ros_ws"
source install/setup.bash
if [ "$SIM_MODE" == "true" ]; then
  ros2 launch nanobot_bringup simulation.launch.py generate_map:="$GENERATE_MAP"
else
  ros2 launch nanobot_bringup robot.launch.py generate_map:="$GENERATE_MAP" use_camera:="$NO_GPU"
fi

trap "echo 'Stopping RealSense Camera container'; sudo docker stop $container_id" EXIT

wait
