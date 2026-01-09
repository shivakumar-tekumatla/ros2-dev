#!/bin/bash

# Start the Docker container
set -e

# Source docker options
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source "$SCRIPT_DIR/docker-options"

# Check if container exists and stop/remove it
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        echo "Container $CONTAINER_NAME is running. Stopping it..."
        docker stop "$CONTAINER_NAME"
    fi
    echo "Removing existing container $CONTAINER_NAME..."
    docker rm "$CONTAINER_NAME"
fi

echo "Starting Docker container: $CONTAINER_NAME"
echo "Mounting host directory: $SCRIPT_DIR/.. to /home/$USER/ros2-dev"
docker run -d --name "$CONTAINER_NAME" -h "$CONTAINER_NAME" -v "$SCRIPT_DIR/../:/home/$USER/ros2-dev" $RUN_OPTIONS "$IMAGE_NAME:$IMAGE_TAG" bash -i -c "while true; do sleep 1; done"

echo "Container started!"
echo "To access the container, run: bash.sh"
