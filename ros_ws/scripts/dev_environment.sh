#!/bin/bash

if [ $# -ne 1 ]; then
    SCRIPT_NAME="$(basename "$0")"
    echo "Usage: $SCRIPT_NAME <new_ip_address> or $SCRIPT_NAME local"
    exit 1
fi

ARG="$1"
CONFIG_FILE="/home/ros_ws/super_client_configuration_file.xml"
UPDATED_CONFIG_FILE="/home/ros_ws/super_client_configuration_file_remote.xml"
BASHRC_FILE="/home/$(whoami)/.bashrc"

if [ "$ARG" == "local" ]; then
    # Remove the updated file and unset the environment variables

    if [ -f "$UPDATED_CONFIG_FILE" ]; then
        rm -f "$UPDATED_CONFIG_FILE"
        echo "Removed updated configuration file."
    fi

    sed -i '/export FASTRTPS_DEFAULT_PROFILES_FILE/d' $BASHRC_FILE
    sed -i '/export ROS_DISCOVERY_SERVER/d' $BASHRC_FILE

    echo "Unset ROS_DISCOVERY_SERVER and FASTRTPS_DEFAULT_PROFILES_FILE."
else
  # Create a configuration file with the new IP address and set the discovery server

  NEW_IP="$ARG"

  if [ ! -f "$CONFIG_FILE" ]; then
      echo "Error: File $CONFIG_FILE not found!"
      exit 1
  fi

  # Replace 127.0.0.1 with the new IP and store it in the script's directory
  sed "s/127.0.0.1/$NEW_IP/g" "$CONFIG_FILE" > "$UPDATED_CONFIG_FILE"

  echo "export FASTRTPS_DEFAULT_PROFILES_FILE=$UPDATED_CONFIG_FILE" >> $BASHRC_FILE
  echo "export ROS_DISCOVERY_SERVER=$NEW_IP:11811" >> $BASHRC_FILE

  echo "FASTRTPS_DEFAULT_PROFILES_FILE set to: $UPDATED_CONFIG_FILE"
  echo "ROS_DISCOVERY_SERVER set to: $NEW_IP:11811"
fi

ros2 daemon stop
ros2 daemon start

echo "Configuration completed. Please restart your terminal to apply the changes."

