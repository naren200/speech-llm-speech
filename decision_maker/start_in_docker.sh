#!/bin/bash
# Source all the necessary environment setup files
source /opt/ros/iron/setup.bash
colcon build
source /root/ros2_ws/install/setup.bash

sleep 2
# Ensure any existing Ollama processes are stopped
pkill ollama || true
# rm -rf ~/.ollama/models  # Fix path to user home directory

# Start Ollama with logging and wait for readiness
ollama serve > ollama.log 2>&1 &
echo "Waiting for Ollama to start..."

# Wait until server responds (max 30 seconds)
timeout=30
while ! curl -s http://localhost:11434 >/dev/null; do
    sleep 1
    timeout=$((timeout-1))
    [ $timeout -le 0 ] && echo "Ollama failed to start!" && exit 1
done

# Pull model with error handling
if ! ollama pull qwen:0.5b; then
    echo "Failed to pull qwen:0.5b, trying default version..."
    ollama pull qwen || {
        echo "Model pull failed. Check available models with: ollama list"
        exit 1
    }
fi


chmod +x /root/ros2_ws/src/speech-llm-speech/decision_maker/ros_process_control.py
cd /root/ros2_ws/; colcon build; 
sleep 6

flask --app /root/ros2_ws/src/speech-llm-speech/decision_maker/ros_process_control.py run --host=0.0.0.0 # --port=5010

#while true; do sleep 2; done
