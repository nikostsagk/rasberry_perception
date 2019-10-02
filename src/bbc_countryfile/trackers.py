#!/usr/bin/env python
from __future__ import absolute_import, division, print_function

import sys

# Import python3 packages after removing ros
__ros_cv2_fix = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if __ros_cv2_fix in sys.path:
    sys.path.remove(__ros_cv2_fix)
    import cv2
    import numpy as np
    from scipy.spatial import distance as dist
    from collections import OrderedDict
    from copy import copy
    from random import randint

    sys.path.append(__ros_cv2_fix)


# tracker_types = ['BOOSTING', 'MIL', 'KCF', 'TLD', 'MEDIANFLOW', 'GOTURN', 'MOSSE', 'CSRT']
# trackers = [cv2.TrackerBoosting_create, cv2.TrackerMIL_create, cv2.TrackerKCF_create, cv2.TrackerTLD_create,
#             cv2.TrackerMedianFlow_create, cv2.TrackerGOTURN_create, cv2.TrackerMOSSE_create, cv2.TrackerCSRT_create]
#
#
# def get_tracker_by_name(tracker_type):
#     # Create a tracker based on tracker name
#     assert tracker_type in tracker_types
#     return trackers[tracker_types.index(tracker_type)]()
#
#
# def get_all_trackers():
#     return {k: get_tracker_by_name(k) for k in tracker_types}


class CentroidTracker:
    def __init__(self, max_disappeared=5):
        self.next_object_id = 0
        self.objects = OrderedDict()
        # self.object_history = OrderedDict()
        self.disappeared = OrderedDict()

        # Max number of frames to miss a detection before deleting a tracklet
        self.max_disappeared = max_disappeared

    def register(self, centroid):
        self.objects[self.next_object_id] = centroid
        self.disappeared[self.next_object_id] = 0
        # self.object_history[self.next_object_id] = [centroid]
        self.next_object_id += 1

    def deregister(self, object_id):
        del self.objects[object_id]
        del self.disappeared[object_id]
        # del self.object_history[object_id]

    def total_tracks(self):
        return self.next_object_id - 1

    def update(self, rects):
        if len(rects) == 0:
            # Update object persistence count and remove if over threshold
            for object_id in list(self.disappeared.keys()):
                self.disappeared[object_id] += 1
                if self.disappeared[object_id] >= self.max_disappeared:
                    self.deregister(object_id)

            return self.objects

        # Update current objects with constant velocity
        # for object_id in list(self.objects.keys()):
        #     if object_id in self.object_history and len(self.object_history) > 1:
        #         oh_id = self.object_history[object_id]
        #         displacement_record = (np.asarray(oh_id) - np.asarray(oh_id[1:] + [oh_id[-1]]))[:-1]
        #         displacement_avg = np.average(displacement_record, axis=0)
        #         print(object_id, self.objects[object_id], '->', end='')
        #         self.objects[object_id] += (
        #                 self.objects[object_id].astype(displacement_avg.dtype) + displacement_avg).astype(
        #             self.objects[object_id].dtype)
        #         print(self.objects[object_id])

        # Calculate centroids
        input_centroids = np.zeros((len(rects), 2), dtype="int")
        for (i, (startX, startY, endX, endY, score, class_id)) in enumerate(rects):
            c_x = (startX + endX) / 2.0
            c_y = (startY + endY) / 2.0
            input_centroids[i] = (c_x, c_y)

        # Track new objects if none are being tracked else match the tracked objects
        if len(self.objects) == 0:
            for i in range(0, len(input_centroids)):
                self.register(input_centroids[i])
        else:
            # Get IDs and centroids of current objects and calculate distance to other tracked objects
            object_ids = list(self.objects.keys())
            object_centroids = list(self.objects.values())
            d = dist.cdist(np.array(object_centroids), input_centroids)
            rows = d.min(axis=1).argsort()
            cols = d.argmin(axis=1)[rows]
            used_rows = set()
            used_cols = set()
            for (row, col) in zip(rows, cols):
                if row in used_rows or col in used_cols:
                    continue
                object_id = object_ids[row]
                self.objects[object_id] = input_centroids[col]
                # self.object_history[object_id].append(input_centroids[col])
                self.disappeared[object_id] = 0
                used_rows.add(row)
                used_cols.add(col)
            unused_rows = set(range(0, d.shape[0])).difference(used_rows)
            unused_cols = set(range(0, d.shape[1])).difference(used_cols)

            # Check if any objects have disappeared else register them as new tracklets
            if d.shape[0] >= d.shape[1]:
                for row in unused_rows:
                    object_id = object_ids[row]
                    self.disappeared[object_id] += 1
                    if self.disappeared[object_id] > self.max_disappeared:
                        self.deregister(object_id)
            else:
                for col in unused_cols:
                    self.register(input_centroids[col])

        return self.objects
