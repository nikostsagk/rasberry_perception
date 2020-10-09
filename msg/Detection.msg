# This message is for storing a single detected object
#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

# Information about the bounding box/mask detection provided by the detection backend
rasberry_perception/RegionOfInterest roi # Represents bounding box detections
rasberry_perception/SegmentOfInterest seg_roi # Represents pixel segmentation (masks)

# Pose information (position of the detected object in the pose_frame_id frame (if blank assume in the camera frame))
string pose_frame_id
geometry_msgs/Pose pose

# Estimated sizes from bbox depth and segm depth
rasberry_perception/ObjectSize size # Size extracted from depth points of bbox (or segm if available)

# ReIdentification Features (some models output descriptors, default is empty list)
float64[] reid_vector
float64[] reid_logits

# Information about the detection (score, class, id etc.)
int64 id # The unique identifier for this particular detection
int64 track_id # The unique identifier for this particular track (0 or -1 if invalid)
float64 confidence # Uncertainty of this detection
string class_name # The unique class name