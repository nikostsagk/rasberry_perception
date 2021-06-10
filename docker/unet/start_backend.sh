#!/bin/bash
set -e

source "/unet_venv/bin/activate" && \
rosrun rasberry_perception detection_server.py --backend unet --model_path epoch_80.pth --config_path param.yaml
#echo 'here3'