# This message is for storing a region of interest (i.e. bounding boxes) in a generic format
#   float64 used over sensor_msgs/RegionOfInterest.msg uint32 to allow for sub-pixel accuracy
#
# If the region of interest only concerns 2D then z1==z2
#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

float64 x1 # Left most pixel of the ROI
float64 y1 # Top most pixel of the ROI
float64 z1 # Front most pixel of the ROI
float64 x2 # Right most pixel of the ROI
float64 y2 # Bottom most pixel of the ROI
float64 z2 # Back most pixel of the ROI