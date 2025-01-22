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


# Forcefully close terminals first
TERMINAL_IDS_FILE="/tmp/ros2_docker_terminals.txt"

if [ -f "$TERMINAL_IDS_FILE" ]; then
    echo "Force-closing terminals..."
    while read -r term_title; do
        if [ -n "$term_title" ]; then
            echo "Killing: $term_title"
            # Force kill terminal and its child processes
            pkill -f "gnome-terminal.*--title=$term_title" 2>/dev/null
            pkill -f "bash.*DEV_TERM_$term_title" 2>/dev/null
            pkill -f "bash.*RUN_TERM_$term_title" 2>/dev/null
        fi
    done < "$TERMINAL_IDS_FILE"
    rm "$TERMINAL_IDS_FILE"
fi

# Stop and remove the Docker Compose services
docker compose down --remove-orphans

echo "Docker Compose services stopped and removed."