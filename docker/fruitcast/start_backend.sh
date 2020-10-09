#!/bin/bash
set -e

# RUN yolov5 backend
source "/fruitcast_venv/bin/activate" && \
rosrun rasberry_perception detection_server.py --backend fruitcast --weights /best.pt
