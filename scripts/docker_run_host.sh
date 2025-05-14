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
  -e DISPLAY=unix:0 \
  -u nano \
  -w /home/nanobot \
  -v "$(pwd)":/home/nanobot \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev/dri:/dev/dri \
  -v /dev/input:/dev/input \
  "$IMAGE_NAME" \
  bash >/dev/null
