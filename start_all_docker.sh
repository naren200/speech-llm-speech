#!/bin/bash

SCRIPT_DIR=$(dirname "$(realpath "$0")")
export LOCAL_ROS_WS="$SCRIPT_DIR"

# API Configuration
export MOCK_MODE=0
export OPENAI_API_KEY="your_key_here"
export HF_API_KEY=""

# Model Configuration
export OLLAMA_MODEL="qwen:0.5b"
export OPENAI_MODEL="gpt-3.5-turbo"
export HUGGINGFACE_MODEL="gpt2"
export LLM_SEQUENCE="{3, 3, 3, 3}"

# Audio Configuration
AUDIO_FILE_NAME="OSR_us_000_0038_8k.wav" #OSR_us_000_0017_8k.wav" # "jfk.wav"
export AUDIO_FILE="/root/ros2_ws/src/speech-llm-speech/whisper_asr/samples/${AUDIO_FILE_NAME}"

# Command line handling
# -------------------------------------------------

# Check for "build" mode -> Example: ./start_docker.sh transcribe --build=true
export DEVELOPER=False
if [ "$1" = "--developer=true" ] || [ "$1" = "--developer=True" ]; then
  export DEVELOPER=True
  echo "Developer mode enabled: Docker images will just be attached."
fi


# Docker compose execution
# -------------------------------------------------
xhost +local:docker

# Start ROS Core first
# docker compose up -d roscore

# Start other services
docker compose up -d --no-deps whisper_asr_node google_tts_node decision_maker_node 

echo -e "\nContainer status:"
docker ps --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}" | grep "_node"

echo "Waiting for containers to be ready..."

./connect_all_containers.sh
