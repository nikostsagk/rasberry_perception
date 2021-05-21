#!/bin/bash
set -e

# RUN unet backend
conda activate torch
rosrun rasberry_perception detection_server.py --backend unet --weights /best.pt
