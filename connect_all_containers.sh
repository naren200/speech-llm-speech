#!/bin/bash

# Function to display an error message and exit
function error_exit {
    echo "$1" 1>&2
    exit 1
}

# Function to get PKG value based on image name
function get_pkg_value {
    local image=$1
    case $image in
        "naren200/google_tts_node:v1")
            echo "/google_tts"
            ;;
        "naren200/whisper_asr_node:v1")
            echo "/whisper_asr"
            ;;
        "naren200/decision_maker_node:v1")
            echo "/decision_maker"
            ;;
        *)
            echo ""
            ;;
    esac
}

# Function to launch terminal for a container
function launch_container_terminal {
    local container_id=$1
    local image=$2
    local name=$3
    local pkg=$(get_pkg_value "$image")
    
    if [ "$DEVELOPER" = "True" ]; then
        gnome-terminal --title="Dev: $name" -- bash -c "docker exec -it $container_id /bin/bash || read -p 'Press Enter to close...'"
    else
        if [ ! -z "$pkg" ]; then
            gnome-terminal --title="$name" -- bash -c "docker exec -it $container_id /bin/bash -l -c \"/bin/bash /root/ros2_ws/src/speech-llm-speech${pkg}/start_in_docker.sh\" || read -p 'Press Enter to close...'"
        fi
    fi
}

# Get running containers
running_containers=$(docker ps --format "{{.ID}}\t{{.Image}}\t{{.Names}}")

# Check if any containers are running
if [ -z "$running_containers" ]; then
    error_exit "No running containers found."
fi

echo "Starting containers in ${DEVELOPER:+developer}${DEVELOPER:-normal} mode"
echo "Container details:"
docker ps --format "table {{.ID}}\t{{.Image}}\t{{.Names}}"

# Launch terminal for each container
while IFS=$'\t' read -r container_id image name; do
    launch_container_terminal "$container_id" "$image" "$name"
    sleep 0.5
done <<< "$running_containers"

echo "Opened terminals for all containers"
