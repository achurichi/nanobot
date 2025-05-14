#!/bin/bash

cd "$(dirname "$0")/.."

IMAGE_NAME="nanobot-dev-image"
CONTAINER_NAME="nanobot-dev"
DOCKERFILE="Dockerfile"

REBUILD=false
RESTART=false

MODE="nano"

# Parse arguments
for arg in "$@"; do
  [[ "$arg" == "rebuild" ]] && REBUILD=true
  [[ "$arg" == "restart" ]] && RESTART=true
  [[ "$arg" == "nano" ]] && MODE="nano"
  [[ "$arg" == "host" ]] && MODE="host"
done

# Functions to check image/container state dynamically
image_exists() {
  docker image inspect "$IMAGE_NAME" >/dev/null 2>&1
}

container_exists() {
  docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"
}

container_is_running() {
  docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"
}

# Build image if not present or rebuild requested
if $REBUILD || ! image_exists; then
  echo "Building Docker image '$IMAGE_NAME'"
  docker build -t "$IMAGE_NAME" \
    --build-arg USERNAME=nano \
    -f "$DOCKERFILE" .
else
  echo "Docker image '$IMAGE_NAME' already exists"
fi

# Remove existing container if rebuild or restart is requested
if { $REBUILD || $RESTART; } && container_exists; then
  echo "Removing existing container '$CONTAINER_NAME'"
  docker rm -f "$CONTAINER_NAME" >/dev/null
fi

# Run container if not running
if ! container_is_running; then
  if container_exists; then
    echo "Container '$CONTAINER_NAME' is stopped, starting it"
    docker start "$CONTAINER_NAME" >/dev/null
  else
    echo "Starting container '$CONTAINER_NAME'"
    if [ "$MODE" == "nano" ]; then
      "$(dirname "$0")/docker_run_nano.sh"
    else
      "$(dirname "$0")/docker_run_host.sh"
    fi
  fi
else
  echo "Container '$CONTAINER_NAME' is already running"
fi

# Attach to the container
echo "Attaching to container"
docker exec -it "$CONTAINER_NAME" /bin/bash
