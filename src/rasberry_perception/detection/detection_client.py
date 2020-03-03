#!/usr/bin/env python

#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

# Executable for sending an image topic to the DetectionServer and publishing the results

from __future__ import absolute_import, division, print_function

import message_filters
import numpy as np
import ros_numpy
import rospy
import tf
from geometry_msgs.msg import PointStamped, Point
from rasberry_perception.msg import Detections
from sensor_msgs.msg import Image, CameraInfo

from rasberry_perception.detection import Client
from rasberry_perception.detection.utility import function_timer




class RunClientOnTopic:
    def __init__(self, image_namespace, depth_namespace=None, score_thresh=0.5):
        stem = image_namespace.split('/')[-1]

        self.namespace = "rasberry_perception/" + stem + "/"
        self.detect_image_topic = self.namespace + "image/image_raw"
        self.visualisation_vis_topic = self.namespace + "image/vis_raw"
        self.detections_topic = self.namespace + "detections"
        self.detections_pose_topic = self.namespace + "pose_array"

        self.score_thresh = score_thresh

        # Wait for connection to detection service
        self.detector = Client()

        # Initialise publishers
        self.detections_pub = rospy.Publisher(self.namespace + "detections", Detections, queue_size=1)
        self.image_pub = rospy.Publisher(self.namespace + "image_raw", Image, queue_size=1)
        self.image_info_pub = rospy.Publisher(self.namespace + "camera_info", CameraInfo, queue_size=1)
        self.detections_vis_pub = rospy.Publisher(self.namespace + "vis_raw", Image, queue_size=1)

        # Initialise subscribers
        subscribers = [
            message_filters.Subscriber(image_namespace + "/image_raw", Image),
            message_filters.Subscriber(image_namespace + "/camera_info", CameraInfo),
        ]

        if depth_namespace is not None:
            raise NotImplementedError("Depth information has to be re-implemented")
            # TODO: Reimplement the 3D pose publisher from 2D detections
            # self.pose_array_image_publisher = PoseArrayPublisher(self.detections_pose_topic)
            # subscribers.extend([
            #     message_filters.Subscriber(depth_namespace + "/image_raw", Image),
            #     message_filters.Subscriber(depth_namespace + "/camera_info", CameraInfo),
            # ])

        # Start subscription
        self.ts = message_filters.ApproximateTimeSynchronizer(subscribers, 10, 0.1)
        self.ts.registerCallback(self.run_detector)

    @function_timer.interval_logger(interval=10)
    def run_detector(self, *args, **kwargs):
        assert len(args) in [2, 4], "Args must either be (colour, info), or (colour, info, depth, info)"
        image_msg, image_info = args[:2]
        result = self.detector(image=image_msg, score_thresh=self.score_thresh)
        if not result.status.OKAY:
            return

        # Publish detection results
        self.detections_pub.publish(result.detections)
        self.image_pub.publish(image_msg)
        self.image_info_pub.publish(image_info)

        # TODO: Reimplement visualisations publisher
        # self.detections_vis_pub.publish(visualisation_image_message)
        # TODO: Reimplement depth pose array publisher


def _get_detections_for_topic():
    rospy.init_node('deep_learning_detector', anonymous=True)

    # get private namespace parameters
    p_image_ns = rospy.get_param('~image_ns', "/pico_zense/colour")
    p_depth_ns = rospy.get_param('~depth_ns', "/pico_zense/aligned_depth_to_colour")
    p_score = rospy.get_param('~score', 0.5)

    rospy.loginfo("Camera Topic to Detection ROS: ")

    # TODO: Re-implement depth for now leave as None
    detector = RunClientOnTopic(image_namespace=p_image_ns, depth_namespace=None, score_thresh=p_score)
    rospy.spin()


if __name__ == '__main__':
    _get_detections_for_topic()
