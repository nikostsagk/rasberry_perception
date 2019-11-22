# This message is for detection results in a generic format
Header header
sensor_msgs/Image rgb
sensor_msgs/Image aligned_depth
sensor_msgs/CameraInfo aligned_depth_info
rasberry_perception/BoundingBox[] bounding_boxes
rasberry_perception/SegmentationLabel[] instances
string[] class_labels