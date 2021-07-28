#!/usr/bin/env bash
set -e
cd "$(cd -P -- "$(dirname -- "$0")" && pwd -P)"

# Identify device
if [ $(uname -m) == "aarch64" ]; then
    export DEVICE=jetson
else
    export DEVICE=gpu
fi;

SERVICE_NAME=$1

export ROS_MASTER_URI="${ROS_MASTER_URI:-"http://localhost:11311/"}"
export ROS_IP="${ROS_IP:-"127.0.0.1"}"
export ROS_HOSTNAME=$ROS_IP

# Function to stop the container on exit sig_(exit int term)
function cleanup {
    docker-compose rm -sf $SERVICE_NAME
    exit 0
}
trap cleanup INT TERM EXIT

# Bring container down if already up (avoid corruption)
docker-compose rm -sf $SERVICE_NAME
# Run backend
docker-compose run $SERVICE_NAME
