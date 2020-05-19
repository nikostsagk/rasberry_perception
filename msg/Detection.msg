# This message is for storing a single detected object
#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

# Information about the bounding box/mask detection provided by the detection backend
rasberry_perception/RegionOfInterest roi # Represents bounding box detections
rasberry_perception/SegmentOfInterest seg_roi # Represents pixel segmentation (masks)

# Pose information (position of the detected object in the pose_frame_id frame (if blank assume in the camera frame))
string pose_frame_id
geometry_msgs/Pose pose

# Information about the detection (score, class, id etc.)
int64 id # The unique identifier for this particular detection
int64 track_id # The unique identifier for this particular track (0 or -1 if invalid)
float64 confidence # Uncertainty of this detection
string class_name # The unique class name