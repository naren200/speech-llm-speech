#!/bin/bash

# Get the directory where this script resides
SCRIPT_DIR=$(dirname "$(realpath "$0")")

# Automatically set ROS workspace to the script's parent directory
export LOCAL_ROS_WS="$SCRIPT_DIR"

# Enable DEVELOPER mode for manual container connection
# export DEVELOPER=True

# Enable mock mode for API testing
export MOCK_MODE=0

# Set the model for the decision_maker node
export OLLAMA_MODEL="gemma2:2b"  # For Ollama model selection
export OPENAI_MODEL="gpt-3.5-turbo"  # For OpenAI model selection
export HUGGINGFACE_MODEL="gpt2"  # For HuggingFace model selection
export LLM_SEQUENCE="{3, 3, 3, 3}"  # For LLM sequence of instances


# Run with specific API key for OpenAI and HuggingFace
export OPENAI_API_KEY=your_key_here
export HF_API_KEY=your_key_here

# Set the audio file for whisper_asr node
export AUDIO_FILE_NAME="jfk.mp3"
export AUDIO_FILE="/root/ros2_ws/src/speech-llm-speech/whisper_asr/samples/${AUDIO_FILE_NAME}"


# Check for "build" mode -> Example: ./start_docker.sh transcribe --build=true
BUILD_MODE=""
if [ "$2" = "--build=true" ] || [ "$2" = "--build=True" ]; then
  BUILD_MODE="--build"
  echo "Build mode enabled: Docker images will be rebuilt."
fi


# Check for "build" mode -> Example: ./start_docker.sh transcribe --build=true
export DEVELOPER=False
if [ "$2" = "--developer=true" ] || [ "$2" = "--developer=True" ]; then
  export DEVELOPER=True
  echo "Developer mode enabled: Docker images will just be attached."
fi


# Requires input parameter: what mode of use_case is this?
# Options:
#     transcribe
#     decide
#     speak

# Check if the provided parameter is one of the allowed values
if [ "$1" != "transcribe" ] && [ "$1" != "decide" ] && [ "$1" != "speak" ]; then
  echo ""
  echo "Error: Invalid mode provided."
  echo ""
  echo "    Options: "
  echo "        transcribe: decodes audio and publishes the decoded transcript"
  echo "        decide: publishes best LLM response for the user query among several sequence of LLM responses"
  echo "        speak: generates audio of the user-defined transcript"
  echo ""
  echo "Usage: $0 <transcribe|decide|speak>"
  echo ""
  exit 1
fi


# Set the environment variable based on the input parameter
COMPUTER_ROLE=$1

# # Export the environment variable so it can be accessed by docker-compose
export COMPUTER_ROLE

PKG="/whisper_asr"
# Start containers based on role
if [ "$COMPUTER_ROLE" == "transcribe" ]; then
  PKG="/whisper_asr"
  docker compose -f docker-compose_separate.yml up -d $BUILD_MODE whisper_asr_node --remove-orphans
elif [ "$COMPUTER_ROLE" == "speak" ]; then
  PKG="/google_tts"
  export LOCAL_ROS_WS="$HOME/Documents/GitHub/speech-llm-speech/google_tts"
  docker compose -f docker-compose_separate.yml up -d $BUILD_MODE google_tts_node --remove-orphans 
elif [ "$COMPUTER_ROLE" == "decide" ]; then
  PKG="/decision_maker"
  docker compose -f docker-compose_separate.yml up -d $BUILD_MODE decision_maker_node --remove-orphans
else
  echo "Invalid role. Use: transcribe, speak, or decide."
  exit 1
fi

export PKG

# Allow local connections to the X server
xhost +local:docker

echo "waiting..."
# docker compose logs

# Adjust the sleep time as necessary based on your container's startup time
sleep 2

echo "connecting..."

# Open a terminal session inside the container
./connect_to_docker.sh

