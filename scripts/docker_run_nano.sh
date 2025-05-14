#!/bin/bash

cd "$(dirname "$0")/.."

IMAGE_NAME="nanobot-dev-image"
CONTAINER_NAME="nanobot-dev"

docker run -dit \
  --name "$CONTAINER_NAME" \
  --privileged \
  --network host \
  --pid host \
  --ipc host \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -u nano \
  -w /home/nanobot \
  -v "$(pwd)":/home/nanobot \
  -v /dev/serial/by-id:/dev/serial/by-id \
  -v /dev/input:/dev/input \
  -v /dev/i2c-0:/dev/i2c-0 \
  -v /dev/i2c-1:/dev/i2c-1 \
  -v /lib/modules/4.9.337-tegra:/lib/modules/4.9.337-tegra \
  -v /var/run/docker.sock:/var/run/docker.sock \
  --runtime=nvidia \
  "$IMAGE_NAME" \
  bash >/dev/null
