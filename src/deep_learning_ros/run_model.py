#!/usr/bin/env python
from __future__ import absolute_import, division, print_function

import sys

# Import python3 packages after removing ros
__ros_cv2_fix = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if __ros_cv2_fix in sys.path:
    sys.path.remove(__ros_cv2_fix)
    import cv2
    import torch
    from mmdet.apis import inference_detector, init_detector, show_result
    from copy import deepcopy
    from timeit import default_timer as timer
    from collections import deque
    from threading import Event

    # Import python2/ros packages after adding ros
    sys.path.append(__ros_cv2_fix)

import ros_numpy
import rospy
from geometry_msgs.msg import PoseArray, Pose, Quaternion, Point
from sensor_msgs.msg import Image, CameraInfo
from yeet.msg import LabelledImage, BoundingBox
# from yeet_pkg.detection import DetectorClient, BBoxProjector
from rasberry_perception_pkg.visualisation import MarkerPublisher
from bbc_countryfile.models import model_lookup
from bbc_countryfile.trackers import CentroidTracker
from std_msgs.msg import String


class DeepLearningRosInference:
    def __init__(self, camera_ns, depth_ns, config_file, checkpoint_file, score_thresh=0.5):
        self.colour_topic = camera_ns + "/image_raw"
        self.colour_info_topic = camera_ns + "/camera_info"
        self.depth_topic = depth_ns + "/image_raw"
        self.depth_info_topic = depth_ns + "/camera_info"

        self.visualisation_topic = camera_ns + "visualisations/image_raw"
        self.score_thresh = score_thresh


        # Initialise detectior
        self.model = init_detector(self.model_config, self.model_path, device=torch.device('cuda', 0))

def deep_learning_ros():
    rospy.init_node('deep_learning_detector', anonymous=True)

    # get private namespace parameters
    p_image_ns = rospy.get_param('~image_ns', "/realsense_camera/color")
    p_depth_ns = rospy.get_param('~depth_ns', "/realsense_camera/aligned_depth_to_color")

    p_config_file = rospy.get_param('~config_path', "")
    p_checkpoint_file = rospy.get_param('~checkpoint_file', "")

    rospy.loginfo("MMDetection ROS: ")

    detector = DeepLearningRosInference(image_ns=p_image_ns, depth_ns=p_depth_ns, config_file=p_config_file,
                                        checkpoint_file=p_checkpoint_file)
    rospy.spin()


if __name__ == '__main__':
    deep_learning_ros()
