#!/bin/bash

# this has to be non-empty for docker-compose to run, even for down
# these don't have to be set to anything correct
export LOCAL_ROS_WS="/"
export COMPUTER_ROLE="/"
export OPENAI_API_KEY="/"
export HF_API_KEY="/"
export OLLAMA_MODEL="/"
export OPENAI_MODEL="/"
export HUGGINGFACE_MODEL="/"
export LLM_SEQUENCE="/"
export MOCK_MODE="/"    
export AUDIO_FILE="/"


# Stop and remove the Docker Compose services, networks, and volumes
docker compose down --remove-orphans
