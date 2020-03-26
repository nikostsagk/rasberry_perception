#!/usr/bin/env python

#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com
from collections import defaultdict
from copy import deepcopy

import cv2
import message_filters
import numpy as np
import pathlib
import ros_numpy
import rospy
from sensor_msgs.msg import Image, CameraInfo

from rasberry_perception.detection.utility import function_timer
from rasberry_perception.msg import TrackerResults, Detections

from std_msgs.msg import String


class TrackLogs:
    def __init__(self):
        # Subscribers
        self.logs = defaultdict(int)

        subscribers = [
            message_filters.Subscriber("/rasberry_perception/colour/image_raw", Image),
            message_filters.Subscriber("/rasberry_perception/colour/camera_info", CameraInfo),
            message_filters.Subscriber("/rasberry_perception/results", Detections),
            message_filters.Subscriber("/rasberry_perception/tracking/results_array", TrackerResults),
        ]

        sync_queue, sync_thresh = 200, 0.1
        rospy.loginfo("Waiting for topics with time synchroniser (queue {}, {}s tolerance) on '{}'".format(
            sync_queue, sync_thresh, ', '.join([s.topic for s in subscribers])
        ))

        self.reset_on = rospy.Subscriber("/sequence_0/info", String, callback=self._dump)
        # self.tracker_results = rospy.Subscriber("/rasberry_perception/tracking/results_array", String, callback=self._dump)
        self.ts = message_filters.ApproximateTimeSynchronizer(subscribers, sync_queue, sync_thresh, allow_headerless=True)
        self.ts.registerCallback(self._track_callback)

    def _dump(self, reset_msg):
        logs = deepcopy(self.logs)
        self.logs = defaultdict(int)
        print("\n".join(["{}: {}".format(k, v) for k, v in logs.items()]))

    @function_timer.logger
    def _track_callback(self, image_msg, camera_info, detector_results, tracker_results):
        fx, fy, cx, cy = camera_info.P[0], camera_info.P[5], camera_info.P[2], camera_info.P[6]
        image = ros_numpy.numpify(image_msg)
        height, width = image.shape[:2]
        size = 128  # TODO: Extract the actual bounding boxes by finding the matching track<->detection
        sizer = size // 2

        dt_boxes = [d.roi for d in detector_results.detections]
        dt_centres = np.asarray([[(r.x2 + r.x1) / 2, (r.y2 + r.y1) / 2] for r in dt_boxes])

        for track in tracker_results.tracks:
            track_output_folder = pathlib.Path(__file__).parent / "data" / str(track.id)
            if not track_output_folder.exists():
                track_output_folder.mkdir()
            x = ((fx * track.pose.position.x) / track.pose.position.z) + cx
            y = ((fy * track.pose.position.y) / track.pose.position.z) + cy

            if x - sizer >= 0 and x + sizer <= width and y - sizer >= 0 and y + sizer <= height:
                track_centre = np.asarray([x, y])
                dist_2 = np.sum((dt_centres - track_centre) ** 2, axis=1)
                closest_dt = np.argmin(dist_2)
                matched_dt = dt_boxes[closest_dt]
                match_cost = dist_2[closest_dt]
                roi_path = track_output_folder / "{:04d}_{}.jpg".format(self.logs[track.id], int(match_cost))
                roi = image[int(matched_dt.y1):int(matched_dt.y2), int(matched_dt.x1):int(matched_dt.x2)]
                cv2.imwrite(str(roi_path), roi)
                self.logs[track.id] += 1
        # print(bounding_boxes)


def __log_unique_tracks():
    rospy.init_node('rasberry_tracking_logs', anonymous=True)
    tracer = TrackLogs()
    rospy.spin()


if __name__ == '__main__':
    __log_unique_tracks()
