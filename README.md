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
ros2 topic pub /text_to_speak std_msgs/msg/String "data: 'Hello, how are you?'" --once
```


```bash
docker exec -it $(docker ps -q) bash
ros2 topic pub /recognized_speech std_msgs/msg/String "data: 'What is the eternity of life?'" --once
```

Talk about Build mode, Developer MOde

docker build -t naren200/decision_maker_node:v1 -f ./Dockerfiles/Dockerfile_decision_maker . 


