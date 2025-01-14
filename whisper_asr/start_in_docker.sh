#!/bin/bash

export MODEL_NAME=download-ggml-model.sh
# Source ROS environment
source /opt/ros/iron/setup.bash

# Check if whisper.cpp exists, if not clone it
cd /root/ros2_ws/src/speech-llm-speech
if [ ! -f "whisper.cpp/CMakeLists.txt" ]; then
    echo "whisper.cpp CMakeLists.txt not found, cloning repository..."
    
    # Clean up if directory exists
    # Stop and remove any processes using whisper.cpp
    lsof +D whisper.cpp/ 2>/dev/null | awk '{print $2}' | tail -n +2 | xargs kill -9 2>/dev/null
    cd ..
    rm -rf whisper.cpp/

    cd /root/ros2_ws/src/speech-llm-speech/
    git clone https://github.com/ggerganov/whisper.cpp.git
    cd whisper.cpp
    # Build whisper.cpp
    mkdir -p build
    cd build
    cmake ..
    make
    cd ..
else
    echo "whisper.cpp exists, skipping clone..."
fi


# Navigate to whisper.cpp directory and build if necessary
cd /root/ros2_ws/src/speech-llm-speech/whisper.cpp
if [ ! -d "build" ]; then
    echo "Creating and building whisper.cpp..."
    mkdir -p build
    cd build
    cmake ..
    make
    cd ../../..
else
    echo "whisper.cpp build directory exists, skipping build..."
fi

# Navigate to whisper_asr directory and check for model
cd /root/ros2_ws/src/speech-llm-speech/whisper_asr
if [ ! -f "model/ggml-base.en.bin" ]; then
    echo "Model not found, downloading..."
    echo "Creating model directory and downloading base model..."
    mkdir -p model
    cd model
    cp ../assets/${MODEL_NAME} .
    sh ./${MODEL_NAME} base.en
else
    echo "Model directory exists, skipping download..."
fi

# Return to workspace root and build ROS packages
cd /root/ros2_ws
rm -rf build/ install/ log/
colcon build

# Source the workspace setup
source /root/ros2_ws/install/setup.bash

sleep 2

# Start Flask application
flask --app /root/ros2_ws/src/speech-llm-speech/whisper_asr/ros_process_control.py run --host=0.0.0.0 --port=5050

#while true; do sleep 2; done
