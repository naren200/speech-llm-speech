#!/bin/bash

export LOCAL_ROS_WS="$HOME/Documents/GitHub/speech-llm-speech"
# export LOCAL_PADMA_ROS_CONFIG_PATH="$HOME/Documents/GitHub/padma-ros-config"
# export LOCAL_PADMA_WEB_UI_PATH="$HOME/Documents/GitHub/padma-web-ui"

export HOME_DIR=$HOME
export DEVELOPER=True

# choose the spray method for testing the spray_type
# 1 - One at a Time.
# 2 - All at once.
# export SERIAL_TYPE_SINGLE_OR_MULTIPLE="1"

# if chosen spray type; choose the range of serial ports to trigger
# export SERIAL_START="0"
# export SERIAL_END="1"

# future functionality could check out specific branches of the inside-the-docker versions of these repos.
# this is commented; for now we mount local copies to the docker, so we choose branches etc. manually.
# this would be used by "setup_in_docker.sh" which is currently not used.
# export PADMA_ROS_CONFIG_BRANCH="feature/distributed_processing"
# export PADMA_WEB_UI_BRANCH="main"

# Requires input parameter: what computer is this?
# Options:
#     SINGLE
#     DISTR_1
#     DISTR_2

# Check if the required parameter is provided
# if [ -z "$1" ]; then
#   echo ""
#   echo "Error: No computer identifier provided."
#   echo ""
#   echo "    Options: "
#   echo "        SINGLE: all nodes on this one computer"
#   echo "        DISTR_1: distributed across two computers; this is the primary, running robot nodes and left camera vision"
#   echo "        DISTR_2: distributed across two computers; this is the secondary, only right camera vision"
#   echo ""
#   echo "Usage: $0 <SINGLE|DISTR_1|DISTR_2>"
#   echo ""
#   exit 1
# fi

# Check if the provided parameter is one of the allowed values
# if [ "$1" != "SINGLE" ] && [ "$1" != "DISTR_1" ] && [ "$1" != "DISTR_2" ]; then
#   echo ""
#   echo "Error: Invalid computer identifier provided."
#   echo ""
#   echo "    Options: "
#   echo "        SINGLE: all nodes on this one computer"
#   echo "        DISTR_1: distributed across two computers; this is the primary, running robot nodes and left camera vision"
#   echo "        DISTR_2: distributed across two computers; this is the secondary, only right camera vision"
#   echo ""
#   echo "Usage: $0 <SINGLE|DISTR_1|DISTR_2>"
#   echo ""
#   exit 1
# fi

# Set the environment variable based on the input parameter
# COMPUTER_ROLE=$1

# # Export the environment variable so it can be accessed by docker-compose
# export COMPUTER_ROLE

# Allow local connections to the X server
xhost +local:docker

# Start the Docker Compose services in the background
docker compose up -d --remove-orphans

echo "waiting..."
# docker compose logs

# Wait a little bit for the container to be fully up and running
# Adjust the sleep time as necessary based on your container's startup time
sleep 2

echo "connecting..."

# Open a terminal session inside the container
./connect_to_docker.sh
