#!/bin/bash
set -e

# Source ROS environment
source "/opt/ros/melodic/setup.bash"
source "/catkin_ws/devel/setup.bash"

# RUN detectron2 backend
source "backend_venv/bin/activate" && \
rosrun rasberry_perception detection_server.py --backend detectron2 --config-file /detectron2/configs/COCO-Detection/fast_rcnn_R_50_FPN_1x.yaml