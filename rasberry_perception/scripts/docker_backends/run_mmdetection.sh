#!/usr/bin/env bash
cd $(cd -P -- "$(dirname -- "$0")" && pwd -P)
image_name="rasberry_perception:mmdetection"
echo "Stopping $image_name"
docker rm $(docker stop $(docker ps -a -q --filter ancestor="$image_name" --format="{{.ID}}")) || echo "$image_name not running"
echo "Starting $image_name"
docker run --network host --gpus all -it $image_name /start