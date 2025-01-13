#!/bin/bash

export LOCAL_ROS_WS="$HOME/Documents/GitHub/speech-llm-speech"
# export LOCAL_PADMA_ROS_CONFIG_PATH="$HOME/Documents/GitHub/padma-ros-config"
# export LOCAL_PADMA_WEB_UI_PATH="$HOME/Documents/GitHub/padma-web-ui"

# export DEVELOPER=True
# Enable mock mode for API testing
export MOCK_MODE=0

# Run with specific API key (if needed)
export OPENAI_API_KEY=your_key_here
export HF_API_KEY=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

export OLLAMA_MODEL="gemma2:2b"  # For Ollama model selection
export OPENAI_MODEL="gpt-3.5-turbo"  # For OpenAI model selection
export HUGGINGFACE_MODEL="gpt2"  # For HuggingFace model selection
export LLM_SEQUENCE="{3,3,3}"  # For three Ollama instances
export MOCK_MODE="false"  # Optional: for testing without actual API calls



# Example: ./start_docker.sh transcribe --build=true

# Check for "build" mode
BUILD_MODE=""
if [ "$2" == "--build=true" ]; then
  BUILD_MODE="--build"
  echo "Build mode enabled: Docker images will be rebuilt."
fi


# # Check only for NVIDIA Container Toolkit in Docker
# if ! docker info | grep -q "Runtimes:.*nvidia"; then
#     echo "Configuring NVIDIA Container Toolkit for Docker..."
    
#     # Configure Docker to use NVIDIA Container Toolkit
#     sudo nvidia-ctk runtime configure --runtime=docker
#     sudo systemctl restart docker
#     sleep 2
#     echo "NVIDIA Container Toolkit configured."
# fi

# Requires input parameter: what computer is this?
# Options:
#     transcribe
#     decide
#     speak

# Check if the provided parameter is one of the allowed values
if [ "$1" != "transcribe" ] && [ "$1" != "decide" ] && [ "$1" != "speak" ]; then
  echo ""
  echo "Error: Invalid computer identifier provided."
  echo ""
  echo "    Options: "
  echo "        transcribe: all nodes on this one computer"
  echo "        decide: distributed across two computers; this is the primary, running robot nodes and left camera vision"
  echo "        speak: distributed across two computers; this is the secondary, only right camera vision"
  echo ""
  echo "Usage: $0 <SINGLE|DISTR_1|DISTR_2>"
  echo ""
  exit 1
fi


# Set the environment variable based on the input parameter
COMPUTER_ROLE=$1

# # Export the environment variable so it can be accessed by docker-compose
export COMPUTER_ROLE

PKG=""
# Start containers based on role
if [ "$COMPUTER_ROLE" == "transcribe" ]; then
  PKG="/whisper_asr"
  docker compose up -d $BUILD_MODE whisper_asr_node --remove-orphans
elif [ "$COMPUTER_ROLE" == "speak" ]; then
  PKG="/google_tts"
  export LOCAL_ROS_WS="$HOME/Documents/GitHub/speech-llm-speech/google_tts"
  docker compose up -d $BUILD_MODE google_tts_node --remove-orphans 
elif [ "$COMPUTER_ROLE" == "decide" ]; then
  PKG="/decision_maker"
  docker compose up -d $BUILD_MODE decision_maker_node --remove-orphans
else
  echo "Invalid role. Use: transcribe, speak, or decide."
  exit 1
fi

export PKG

# Allow local connections to the X server
xhost +local:docker

# # Start the Docker Compose services in the background
# docker compose up -d --remove-orphans

echo "waiting..."
# docker compose logs

# Wait a little bit for the container to be fully up and running
# Adjust the sleep time as necessary based on your container's startup time
sleep 2

echo "connecting..."

# Open a terminal session inside the container
./connect_to_docker.sh

