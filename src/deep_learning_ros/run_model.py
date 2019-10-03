#!/usr/bin/env python
from __future__ import absolute_import, division, print_function

import message_filters
import ros_numpy
import rospy
from sensor_msgs.msg import Image, CameraInfo

from deep_learning_ros.compatibility_layer.detection_server import DetectorResultsClient, DETECTOR_OK
from rasberry_perception_pkg.utility import function_timer


class DeepLearningRosInference:
    def __init__(self, colour_ns, depth_ns, config_file, checkpoint_file, score_thresh=0.5):
        self.colour_topic = colour_ns + "/image_raw"
        self.colour_info_topic = colour_ns + "/camera_info"
        self.depth_topic = depth_ns + "/image_raw"
        self.depth_info_topic = depth_ns + "/camera_info"

        self.visualisation_topic = colour_ns + "visualisations/image_raw"
        self.score_thresh = score_thresh

        # Wait for connection to detection service
        self.detector = DetectorResultsClient()

        # Initialise subscribers
        self.colour_sub = message_filters.Subscriber(self.colour_topic, Image)
        self.depth_sub = message_filters.Subscriber(self.depth_topic, Image)
        self.depth_info_sub = message_filters.Subscriber(self.depth_info_topic, CameraInfo)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.colour_sub, self.depth_sub, self.depth_info_sub], 10, 0.5)

        self.ts.registerCallback(self.run_detector)

    def on_frame(self, colour_msg, depth_msg, depth_info_msg):
        self.run_detector(colour_msg, depth_msg, depth_info_msg)

    @function_timer.logger
    def run_detector(self, colour_msg, depth_msg, depth_info_msg):
        detection_result = self.detector(colour_msg)
        print("detector", detection_result.status)
        if detection_result != DETECTOR_OK:
            return

        rgb_image = ros_numpy.numpify(colour_msg)
        depth_image = ros_numpy.numpify(depth_msg)


def deep_learning_ros():
    rospy.init_node('deep_learning_detector', anonymous=True)

    # get private namespace parameters
    p_image_ns = rospy.get_param('~image_ns', "/camera/color")
    p_depth_ns = rospy.get_param('~depth_ns', "/camera/aligned_depth_to_color")

    p_config_file = rospy.get_param('~config_path',
                                    "/home/raymond/projects/mmdetection/configs/bbc_countryfile/grid_rcnn_gn_head_r50_fpn_1x_2cls_1333x800.py")
    p_checkpoint_file = rospy.get_param('~checkpoint_file',
                                        "/home/raymond/projects/mmdetection/work_dirs/bbc_countryfile/grid_rcnn_gn_head_r50_fpn_1x_2cls_1333x800/epoch_12.pth")

    rospy.loginfo("MMDetection ROS: ")

    detector = DeepLearningRosInference(colour_ns=p_image_ns, depth_ns=p_depth_ns, config_file=p_config_file,
                                        checkpoint_file=p_checkpoint_file)
    rospy.spin()


if __name__ == '__main__':
    deep_learning_ros()
