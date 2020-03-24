#!/usr/bin/env python

#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

from __future__ import absolute_import, division, print_function

import rospy

import numpy as np

from scipy.optimize import linear_sum_assignment

from rasberry_perception.msg import Detections

from rasberry_perception.tracking.sort import Sort


class ObjectTrackerNode:
    def __init__(self, detection_topic, tracker_topic, cost_threshold, max_age, min_hits):
        self.tracker = Sort(max_age=max_age, min_hits=min_hits)

        self.cost_threshold = cost_threshold

        # Advertise the result of Object Tracker
        self.pub_trackers = rospy.Publisher(tracker_topic, Detections, queue_size=1)

        self.sub_detection = rospy.Subscriber(detection_topic, Detections, self.detection_callback)

    def detection_callback(self, pose_array_msg):
        detection_list_xywhs = []

        for detection in pose_array_msg.detections:
            r = detection.roi
            x, y = (r.x2 + r.x1) / 2, (r.y2 + r.y1) / 2
            detection_list_xywhs.append([x, y, r.x2-r.x1, r.y2-r.y1, detection.confidence * 100])

        detection_list_xywhs = np.asarray(detection_list_xywhs)

        # Call the tracker
        tracks = self.tracker.update(detection_list_xywhs)

        # Copy the detections
        detections_copy = pose_array_msg.detections

        tracked_detections = []

        if len(detection_list_xywhs) > 0:
            print(tracks)
            # Create cost matrix
            # Double for in Python :(
            C = np.zeros((len(tracks), len(detection_list_xywhs)))
            for i, track in enumerate(tracks):
                for j, det in enumerate(detection_list_xywhs):
                    C[i, j] = np.linalg.norm(det[0:-1] - track[0:-1])

            # apply linear assignment
            row_ind, col_ind = linear_sum_assignment(C)

            for i, j in zip(row_ind, col_ind):
                if C[i, j] < self.cost_threshold and j != 0:
                    id = "id={}".format(tracks[i, 4])
                    print("{} -> {} with cost {}". format(id, detections_copy[j-1].track_id, C[i, j]))
                    trk_bbox = detections_copy[j-1]
                    trk_bbox.track_id = id
                    tracked_detections.append(trk_bbox)

            print("---")
        else:
            print("No tracked objects!")

        pose_array_msg.detections = tracked_detections
        self.pub_trackers.publish(pose_array_msg)


def __tracking_from_detections():
    # raise NotImplementedError("Not currently supported in favour of BayesianTracking project for CVPR.")
    rospy.init_node('rasberry_perception_sort_tracker')

    detection_topic = rospy.get_param("~detection_topic", "/rasberry_perception/results")
    tracker_topic = rospy.get_param('~tracker_topic', "/rasberry_perception/results_tracked")
    cost_threshold = rospy.get_param('~cost_threshold', 50)
    min_hits = rospy.get_param('~min_hits', 1)
    max_age = rospy.get_param('~max_age', 5)

    ObjectTrackerNode(detection_topic, tracker_topic, cost_threshold, max_age, min_hits)
    rospy.spin()


if __name__ == '__main__':
    __tracking_from_detections()
