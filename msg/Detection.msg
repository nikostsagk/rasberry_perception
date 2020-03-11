# This message is for storing a single object detection
#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

# Information about the bounding box/mask detection provided by the detection backend
rasberry_perception/RegionOfInterest roi # represents bounding box detections
rasberry_perception/SegmentOfInterest seg_roi # represents pixel segmentations (masks)

# Information about the detection (score, class, id etc.)
int64 id # the unique identifier for this particular detection
float64 confidence # uncertainty of this detection
string class_name # the unique class name