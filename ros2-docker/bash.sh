#!/bin/bash

# Launch bash shell in the Docker container
set -e

# Source docker options
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source "$SCRIPT_DIR/docker-options"

echo "Launching bash in $CONTAINER_NAME..."
docker exec -it "$CONTAINER_NAME" bash
