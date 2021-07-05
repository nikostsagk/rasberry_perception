#!/bin/bash
set -e

# RUN detectron2 backend
source "/detectron2_venv/bin/activate" && \
rosrun rasberry_perception detection_server.py --backend detectron2 --config_file config.yaml --model_file model.pth --service_name $SERVICE_NAME
