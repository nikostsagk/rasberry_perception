#!/usr/bin/env python

#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com
from collections import defaultdict
from copy import deepcopy

import pathlib
import ros_numpy
import rospy
from std_msgs.msg import String

import cv2
import numpy as np

from rasberry_perception.detection.utility import function_timer
from rasberry_perception.msg import Detections
from threading import Event


class TrackLogs:
    def __init__(self, bbox_save_dir="", reset_on="", plot_tracks_file=""):
        # Subscribers
        self.logs = defaultdict(int)

        if reset_on:
            self.reset_on = rospy.Subscriber(reset_on, String, callback=self._dump)

        if bbox_save_dir:
            self.save_dir = bbox_save_dir
            self.write_bbox_sub = rospy.Subscriber("/rasberry_perception/tracking/detection_results_array",
                                                   Detections, self._bbox_write_callback, queue_size=200)
        self._plot = bool(plot_tracks_file)
        try:
            import matplotlib
            matplotlib.use("TkAgg")
            import matplotlib.pyplot as plt
        except ImportError:
            print("Please install matplotlib to enable plotting")
            self._plot = False

        if self._plot:
            self.canvas_history = 0
            self.tracks_history = defaultdict(lambda: dict(x=[], y=[]))
            self.fig, self.ax = plt.subplots()
            self.fig.set_tight_layout(True)
            self.fig_updated = Event()
            self.plot_tracks_sub = rospy.Subscriber("/rasberry_perception/tracking/detection_results_array",
                                                    Detections, self._plot_tracks_callback, queue_size=200)

    def _dump(self, reset_msg):
        self.canvas_history = 0
        self.logs = defaultdict(int)
        self.tracks_history = defaultdict(lambda: dict(x=[], y=[]))

    def spin(self):
        if not self._plot:
            rospy.spin()
        while not rospy.is_shutdown():
            if self._plot:
                if self.fig_updated.is_set():
                    self.fig_updated.clear()
                self.plt_show()

    @function_timer.logger
    def _bbox_write_callback(self, tracker_detections):
        image_msg = tracker_detections.camera_frame

        image = ros_numpy.numpify(image_msg)
        height, width = image.shape[:2]

        for track in tracker_detections.objects:
            track_output_folder = pathlib.Path(__file__).parent / self.save_dir / str(track.track_id)
            if not track_output_folder.exists():
                track_output_folder.mkdir()

            if track.roi.x1 >= 0 and track.roi.x2 <= width and track.roi.y1 >= 0 and track.roi.y2 <= height:
                roi_path = track_output_folder / "{:04d}.jpg".format(self.logs[track.track_id])
                roi = image[int(track.roi.y1):int(track.roi.y2), int(track.roi.x1):int(track.roi.x2)]
                cv2.imwrite(str(roi_path), roi)
                self.logs[track.track_id] += 1
        # print(bounding_boxes)

    def plt_show(self):
        if not self._plot:
            return
        plt.cla()

        self.ax.set_xlim([0, 1280])
        self.ax.set_ylim([0, 720])

        # Plot line histories
        for track_id, track_data in self.tracks_history.items():
            self.ax.plot(track_data["x"], track_data["y"], label=str(track_id))

        # self.ax.legend(ncol=len(self.tracks_history))

        self.fig.canvas.draw()
        plt.pause(0.001)

    @function_timer.logger
    def _plot_tracks_callback(self, tracker_detections):
        image_msg = tracker_detections.camera_frame
        fx, fy, cx, cy = tracker_detections.camera_info.P[0], tracker_detections.camera_info.P[5], \
                         tracker_detections.camera_info.P[2], tracker_detections.camera_info.P[6]

        image = ros_numpy.numpify(image_msg)
        height, width = image.shape[:2]

        # Add to track histories
        for track in tracker_detections.objects:
            x = ((fx * track.pose.position.x) / (track.pose.position.z + np.finfo(float).eps)) + cx
            y = ((fy * track.pose.position.y) / (track.pose.position.z + np.finfo(float).eps)) + cy
            if 0 <= x <= width and 0 <= y <= height:
                self.tracks_history[track.track_id]["x"].append(x)
                self.tracks_history[track.track_id]["y"].append(y)

        # self.ax.imshow(image)
        if self._plot:
            self.fig_updated.set()
            print(self.canvas_history)
            self.canvas_history += 1


def __log_unique_tracks():
    rospy.init_node('rasberry_tracking_logs', anonymous=True)
    reset_on = "/sequence_0/info"
    bbox_save_dir = "data"
    plot_tracks_file = ""  # "tracks.gif"
    tracer = TrackLogs(bbox_save_dir, reset_on, plot_tracks_file)
    tracer.spin()


if __name__ == '__main__':
    __log_unique_tracks()
