#!/bin/bash
set -e

# RUN detectron2 backend
source "backend_venv/bin/activate" && \
rosrun rasberry_perception detection_server.py --backend detectron2 --config-file /r50_packaged/config.yaml
