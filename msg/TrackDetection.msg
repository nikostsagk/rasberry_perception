# This message stores information about a tracked object (extension of TaggedPose)
#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

int64 id # The track-id or detection identifier
string tag # The image tag (used with '*-LABELLED' matching algorithms, blank if tags not used)
geometry_msgs/Pose pose # The predicted pose