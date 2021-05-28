#!/bin/bash
set -e

# RUN unet backend
conda activate torch && \
python /berry_segmentation/import_test.py
#rosrun rasberry_perception detection_server.py --backend unet --weights /best.pt
