#!/bin/bash

cd "$(dirname "$0")/.."

# Build ros packages
colcon build --symlink-install
