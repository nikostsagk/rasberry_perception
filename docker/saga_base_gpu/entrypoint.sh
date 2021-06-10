#!/bin/bash --login
set -e
source "/opt/ros/melodic/setup.bash"
source "/catkin_ws/devel/setup.bash"
{
    echo "Try starting detection backend..."
    set -euo pipefail
    conda activate torch
    source "/start_backend.sh" 2> /dev/null
} || {
    echo "No ebackend configured."
    exec "$@"
}