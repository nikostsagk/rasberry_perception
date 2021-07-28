#!/bin/bash
set -e

# RUN tensorrtdeepsort backend
source "modularmot_venv/bin/activate" && \
rosrun rasberry_perception detection_server.py --backend tensorrtdeepsort --config_path config.json --service_name $SERVICE_NAME
