# This message is a complete snapshot of detections on an camera frame
#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

# The original message sources the detections are based on
sensor_msgs/Image camera_frame # The original sensor_msgs/Image message the detections are calculated from
sensor_msgs/CameraInfo camera_info # The original sensor_msgs/CameraInfo message the detections are calculated from

# Contains the results of the detection server backend
rasberry_perception/Detection[] objects # Detection results for the image