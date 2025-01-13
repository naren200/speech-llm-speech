# speech-llm-speech



# google_tts

```bash
./start_docker.sh speak

colcon build
source install/setup.bash
ros2 run google_tts google_tts
```

```bash
docker exec -it $(docker ps -q) bash
ros2 topic pub /text_to_speak std_msgs/msg/String "data: 'Hello, how are you?'"
```

Talk about Build mode, Developer MOde

