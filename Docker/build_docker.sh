#!/bin/bash

echo "UID=$(id -u)"
echo "GID=$(id -g)"

set -e

cd "$(dirname "$0")/../.."

echo "Building ROS Noetic Docker image..."
docker build -t noetic:amr_navigation \
    --build-arg USER=$USER \
    --build-arg UID=$(id -u) \
    --build-arg GID=$(id -g) \
    -f amr_navigation/Docker/Dockerfile .

echo "Build completed successfully!"