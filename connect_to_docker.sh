# Function to display an error message and exit
function error_exit {
    echo "$1" 1>&2
    exit 1
}

# Check if DEVELOPER is set
if [ "$DEVELOPER" = "True" ]; then
    echo "DEVELOPER mode detected."

    # Get the list of running container IDs
    running_containers=$(docker ps -q)

    # Check if any containers are running
    if [ -z "$running_containers" ]; then
        error_exit "No running containers found."
    fi

    # Handle single or multiple running containers
    container_count=$(echo "$running_containers" | wc -l)
    if [ "$container_count" -eq 1 ]; then
        # Only one container running, connect to it
        container_id=$(echo "$running_containers")
        echo "Connecting to the running container (ID: $container_id)..."
        docker exec -it "$container_id" /bin/bash || error_exit "Failed to connect to the container."
    elif [ "$container_count" -gt 1 ]; then
        # Multiple containers running, request container ID
        echo "Multiple containers are running:"
        docker ps --format "table {{.ID}}\t{{.Image}}\t{{.Names}}"
        echo "Please specify the container ID to connect to:"
        read -r container_id
        echo "Connecting to the specified container (ID: $container_id)..."
        docker exec -it "$container_id" /bin/bash || error_exit "Failed to connect to the container."
    fi
else
    echo "Non-DEVELOPER mode detected."

    # Get the list of running container IDs
    running_containers=$(docker ps -q)

    # Check if any containers are running
    if [ -z "$running_containers" ]; then
        error_exit "No running containers found."
    fi

    # Handle single or multiple running containers
    container_count=$(echo "$running_containers" | wc -l)
    if [ "$container_count" -eq 1 ]; then
        # Only one container running, connect to it
        container_id=$(echo "$running_containers")
        echo "Connecting to the running container and executing startup script (ID: $container_id)..."
        docker exec -it "$container_id" /bin/bash -l -c "/bin/bash src/speech-llm-speech/start_in_docker.sh" || error_exit "Failed to execute startup script in the container."
    elif [ "$container_count" -gt 1 ]; then
        # Multiple containers running, request container ID
        echo "Multiple containers are running:"
        docker ps --format "table {{.ID}}\t{{.Image}}\t{{.Names}}"
        echo "Please specify the container ID to connect to:"
        read -r container_id
        echo "Connecting to the specified container and executing startup script (ID: $container_id)..."
        docker exec -it "$container_id" /bin/bash -l -c "/bin/bash src/speech-llm-speech/start_in_docker.sh" || error_exit "Failed to execute startup script in the container."
    fi
fi

