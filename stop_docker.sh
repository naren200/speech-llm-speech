#!/bin/bash

# this has to be non-empty for docker-compose to run, even for down
# these don't have to be set to anything correct
export LOCAL_ROS_WS="/"
export COMPUTER_ROLE="/"

#
# Stop and remove the Docker Compose services, networks, and volumes
docker compose down --remove-orphans
