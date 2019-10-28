#!/usr/bin/env python
from __future__ import absolute_import, division, print_function

import rospy

import numpy as np

from scipy.optimize import linear_sum_assignment

from rasberry_perception.msg import ImageDetections

from tracking.sort import sort


class ObjectTrackerNode:
    def __init__(self, detection_topic, tracker_topic, cost_threshold, max_age, min_hits):
        self.tracker = sort.Sort(max_age=max_age, min_hits=min_hits)

        self.cost_threshold = cost_threshold

        # Advertise the result of Object Tracker
        self.pub_trackers = rospy.Publisher(tracker_topic, ImageDetections, queue_size=1)

        self.sub_detection = rospy.Subscriber(detection_topic, ImageDetections, self.detection_callback)

    def detection_callback(self, detection_msg):
        detection_list_xywhs = []
        bounding_boxes = detection_msg.bounding_boxes

        for b_box in bounding_boxes:
            x1, y1, x2, y2, score = b_box.x1, b_box.y1, b_box.x2, b_box.y2, b_box.score
            w, h = x2 - x1, y2 - y1
            x, y = (x2 + x1) / 2, (y2 + y1) // 2
            detection_list_xywhs.append([x, y, w+x, h+y, score])

        detection_list_xywhs = np.asarray(detection_list_xywhs)



        # Call the tracker
        tracks = self.tracker.update(detection_list_xywhs)

        # Copy the detections
        bounding_boxes_copy = bounding_boxes

        tracked_bounding_boxes = []

        if len(detection_list_xywhs) > 0:
            print(tracks)
            # Create cost matrix
            # Double for in Python :(
            C = np.zeros((len(tracks), len(detection_list_xywhs)))
            for i, track in enumerate(tracks):
                for j, det in enumerate(detection_list_xywhs):
                    C[i, j] = np.linalg.norm(det[0:-2] - track[0:-2])

            # apply linear assignment
            row_ind, col_ind = linear_sum_assignment(C)

            for i, j in zip(row_ind, col_ind):
                if C[i, j] < self.cost_threshold and j != 0:
                    id = "id={}".format(tracks[i, 4])
                    print("{} -> {} with cost {}". format(id, bounding_boxes_copy[j-1].b_box_id, C[i, j]))
                    trk_bbox = bounding_boxes_copy[j-1]
                    trk_bbox.b_box_id = id
                    tracked_bounding_boxes.append(trk_bbox)

            print("---")
        else:
            print("No tracked objects!")

        detection_msg.bounding_boxes = tracked_bounding_boxes
        self.pub_trackers.publish(detection_msg)


def __tracking_from_detections():
    rospy.init_node('rasberry_perception_detection_tracker')

    detection_topic = rospy.get_param("~detection_topic", "/detection/predictions")
    tracker_topic = rospy.get_param('~tracker_topic', "/detection/tracks")
    cost_threshold = rospy.get_param('~cost_threshold', 15)
    min_hits = rospy.get_param('~min_hits', 1)
    max_age = rospy.get_param('~max_age', 5)

    ObjectTrackerNode(detection_topic, tracker_topic, cost_threshold, max_age, min_hits)
    rospy.spin()


if __name__ == '__main__':
    __tracking_from_detections()
