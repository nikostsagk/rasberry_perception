#!/usr/bin/env bash
cd $(cd -P -- "$(dirname -- "$0")" && pwd -P)
image_name="rasberry_perception:$1"
echo "Starting $image_name"
docker run --network host --gpus all --name $1_backend --rm -it $image_name