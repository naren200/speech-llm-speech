#!/bin/bash
# Source all the necessary environment setup files
source /opt/ros/iron/setup.bash
colcon build
source /root/ros2_ws/install/setup.bash

# git config --global --add safe.directory /root/ros2_ws/src/agrobot
# git config --global --add safe.directory /root/ros2_ws/src/padma-ros-config
# git config --global --add safe.directory /root/ros2_ws/src/padma-web-ui

sleep 2

# Get the local IP address
# LOCAL_IP=$(hostname -I | awk '{print $1}')

# Define the path to your config.yaml
# CONFIG_FILE="src/padma-web-ui/config.yaml"
# export PADMA_INFERENCE_MODEL=cropsweeds_0.7.onnx

# Write the local IP address to config.yaml
# echo "API_URL: \"http://$LOCAL_IP\"" > $CONFIG_FILE

#printenv
# Run the Flask application
# cd src/padma-web-ui/; python3 -m http.server 8080 &
cd /root/ros2_ws/; colcon build; 
flask --app /root/ros2_ws/src/speech-llm-speech/google_tts/ros_process_control.py run --host=0.0.0.0 --port=5050

#while true; do sleep 2; done
