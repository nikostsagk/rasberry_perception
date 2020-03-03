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
from geometry_msgs.msg import PointStamped, Point, PoseArray
from rasberry_perception.msg import Detections
from sensor_msgs.msg import Image, CameraInfo

from rasberry_perception.detection import Client, default_service_name
from rasberry_perception.detection.utility import function_timer




class RunClientOnTopic:
    def __init__(self, image_namespace, depth_namespace=None, score_thresh=0.5, service_name=default_service_name):
        self._node_name = service_name + "_client"
        rospy.init_node(self._node_name, anonymous=True)

        stem = image_namespace.split('/')[-1]

        self.namespace = "rasberry_perception/" + stem + "/"

        self.score_thresh = score_thresh

        # Wait for connection to detection service
        self.detector = Client()

        # Initialise publishers
        self.detections_pub = rospy.Publisher(self.namespace + "detections", Detections, queue_size=1)
        self.image_pub = rospy.Publisher(self.namespace + "colour/image_raw", Image, queue_size=1)
        self.image_info_pub = rospy.Publisher(self.namespace + "colour/camera_info", CameraInfo, queue_size=1)
        self.detections_vis_pub = rospy.Publisher(self.namespace + "colour/vis_raw", Image, queue_size=1)

        # Initialise subscribers
        subscribers = [
            message_filters.Subscriber(image_namespace + "/image_raw", Image),
            message_filters.Subscriber(image_namespace + "/camera_info", CameraInfo),
        ]

        self.depth_enabled = depth_namespace is not None
        if self.depth_enabled:
            self.depth_pub = rospy.Publisher(self.namespace + "depth/image_raw", Image, queue_size=1)
            self.depth_info_pub = rospy.Publisher(self.namespace + "depth/camera_info", CameraInfo, queue_size=1)
            self.depth_bbox_detections_pub = rospy.Publisher(self.namespace + "bbox_detections", PoseArray,
                                                             queue_size=1)
            self.depth_segm_detections_pub = rospy.Publisher(self.namespace + "segm_detections", PoseArray,
                                                             queue_size=1)

            subscribers.extend([
                message_filters.Subscriber(depth_namespace + "/image_raw", Image),
                message_filters.Subscriber(depth_namespace + "/camera_info", CameraInfo),
            ])

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

        if self.depth_enabled and len(args) == 4:
            depth_msg, depth_info = args[2:]
            for detection in result.detections:
                pass

        # Publish detection results
        self.detections_pub.publish(result.detections)
        self.image_pub.publish(image_msg)
        self.image_info_pub.publish(image_info)

        # TODO: Reimplement visualisations publisher
        # self.detections_vis_pub.publish(visualisation_image_message)
        # TODO: Reimplement depth pose array publisher


def _get_detections_for_topic():
    # get private namespace parameters
    p_image_ns = rospy.get_param('~image_ns', "/pico_zense/colour")
    p_depth_ns = rospy.get_param('~depth_ns', "/pico_zense/aligned_depth_to_colour")
    p_score = rospy.get_param('~score', 0.5)

    rospy.loginfo("Camera Topic to Detection ROS: image_namespace={}, depth_namespace={}, score_thresh={}".format(
        p_image_ns, p_depth_ns, p_score
    ))

    # TODO: Re-implement depth for now leave as None
    detector = RunClientOnTopic(image_namespace=p_image_ns, depth_namespace=p_depth_ns, score_thresh=p_score)
    rospy.spin()


if __name__ == '__main__':
    _get_detections_for_topic()
