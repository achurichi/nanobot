#!/bin/bash

cd "/root/ros_ws"

TARGET_PACKAGES=$(colcon list --base-paths src/nanobot --names-only)

if [[ "$1" == "all" ]]; then
  colcon build --symlink-install --packages-up-to $TARGET_PACKAGES
else
  colcon build --symlink-install --packages-select $TARGET_PACKAGES
fi