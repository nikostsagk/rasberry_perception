#!/bin/bash --login
set -e
source "/opt/ros/melodic/setup.bash"
source "/catkin_ws/devel/setup.bash"
{
    echo "Try starting detection backend..."
    source "/start_backend.sh" 2> /dev/null
} || {
    echo "No backend configured."
    exec "$@"
}