#!/bin/bash

# Build the Docker image
set -e

# Source docker options
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source "$SCRIPT_DIR/docker-options"

echo "Building Docker image: $IMAGE_NAME:$IMAGE_TAG"
echo "Using user: $USER"
docker build --build-arg USER=$USER $BUILD_ARGS -t "$IMAGE_NAME:$IMAGE_TAG" .

echo "Build complete!"
