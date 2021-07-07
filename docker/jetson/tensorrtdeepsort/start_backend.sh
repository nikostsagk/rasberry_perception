#!/bin/bash
set -e

# RUN yolov4deepsort backend
rosrun rasberry_perception detection_server.py --backend tensorrtdeeposrt --configPath /catkin_ws/src/Noronn/deep_sort_yolov4-master/cfg/yolov4_sb.cfg
