#!/bin/bash
# Source all the necessary environment setup files
source /opt/ros/iron/setup.bash
colcon build
source /root/ros2_ws/install/setup.bash

sleep 2
chmod +x /root/ros2_ws/src/speech-llm-speech/decision_maker/ros_process_control.py
cd /root/ros2_ws/; colcon build; 
flask --app /root/ros2_ws/src/speech-llm-speech/decision_maker/ros_process_control.py run --host=0.0.0.0 --port=5050

#while true; do sleep 2; done
