#!/bin/bash
set -e

# RUN detectron2 backend
source "/detectron2_venv/bin/activate" && \
rosrun rasberry_perception detection_server.py --backend detectron2 --config_file detectron2/configs/COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml --model_file model_final.pth --service_name gripper_perception
