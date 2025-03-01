#!/bin/bash

cd "$(dirname "$0")/.."

# Get third party repos
vcs import src < third_parties.repos

# Build ros packages
colcon build --symlink-install
