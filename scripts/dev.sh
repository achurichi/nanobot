#!/bin/bash

cd "$(dirname "$0")/.."

# Default values
SOURCE_PATH=$(pwd)
REMOTE_PATH=""
SSH_TARGET=""
MODE="host"

forward_args=()

for arg in "$@"; do
  if [[ "$arg" =~ ^[^@]+@[^:]+:.+ ]]; then
    # Argument is in the format <user>@<ip>:<path>
    MODE="nano"
    SSH_TARGET="${arg%%:*}" # Extracts <user>@<ip>
    REMOTE_PATH="${arg#*:}" # Extracts <path>
  elif [[ "$arg" == "rebuild" || "$arg" == "restart" ]]; then
    forward_args+=("$arg")
  fi
done

if [[ "$MODE" == "nano" ]]; then
  echo "Setting up file watcher"

  # Check if .venv exists
  if [ ! -d ".venv" ]; then
    echo "Creating Python virtual environment and installing watchdog"
    sudo apt-get install -y python3-pip python3-venv
    python3 -m venv .venv
    source .venv/bin/activate
    pip3 install watchdog
  fi

  RSYNC_CMD="rsync -avz --delete \
  --exclude=ros_ws/build \
  --exclude=ros_ws/install \
  --exclude=ros_ws/log \
  --exclude=web_app/node_modules \
  $SOURCE_PATH/ \
  $SSH_TARGET:$REMOTE_PATH/"

  # Initial sync
  eval "$RSYNC_CMD" >/dev/null

  # Check if watchmedo is already running
  # TODO: Avoid collisions with watchmedo processes from different sources
  if ! pgrep -x "watchmedo" >/dev/null; then
    # Start watchmedo in background if it's not running
    watchmedo shell-command \
      --drop \
      --patterns="*" \
      --recursive \
      --command="$RSYNC_CMD" \
      "$SOURCE_PATH/" >/dev/null 2>&1 &
    WATCHMEDO_PID=$!
    echo "File sync running in background (PID $WATCHMEDO_PID)"
  else
    echo "File sync already running, skipping start"
  fi

  # Connect and run docker_launch.sh with the forwarded arguments
  echo "Connecting to $SSH_TARGET"
  ssh -t "$SSH_TARGET" "$REMOTE_PATH/scripts/docker_launch.sh nano ${forward_args[*]}"

  # Only stop watchmedo if the process was started
  trap 'if [ -n "$WATCHMEDO_PID" ]; then echo "Stopping file sync"; kill "$WATCHMEDO_PID"; fi' EXIT
else
  # Run locally with "host" mode
  ./scripts/docker_launch.sh host "${forward_args[@]}"
fi
