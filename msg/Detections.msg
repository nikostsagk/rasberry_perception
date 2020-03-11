# This message is an image with corresponding detections
#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

# The header.frame_id should represent the frame used in the detections field (below)
Header header # Detection origin header i.e. the rgb header the detections are from

# Contains the results of the detection server backend
rasberry_perception/Detection[] detections # Detection results for the image