#!/usr/bin/env python

#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com
from collections import defaultdict

import rospy
from rasberry_perception.msg import TrackerResults

from std_msgs.msg import String


class TrackLogs:
    def __init__(self, people_tracker_topic=None):
        # Subscribers
        self.logs = defaultdict(list)
        if people_tracker_topic is None:
            people_tracker_topic = '/rasberry_perception/tracking/results_array'
        self.tracker_results_subscriber = rospy.Subscriber(people_tracker_topic, TrackerResults, self._track_callback)


    def _track_callback(self, tracker_results_msg):
        for track in tracker_results_msg.tracks:
            self.logs[track.id].append(track)

        print(self.logs.keys())


def __log_unique_tracks():
    rospy.init_node('rasberry_tracking_logs', anonymous=True)
    tracker_topic = rospy.get_param("~tracker_topic", None)
    counter = TrackLogs(tracker_topic)
    rospy.spin()


if __name__ == '__main__':
    __log_unique_tracks()
