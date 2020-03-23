#!/usr/bin/env python

#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

import rospy
from rasberry_perception.msg import TrackerResults

from std_msgs.msg import String


class CountAnnouncer:
    def __init__(self, people_tracker_topic=None, reset_topic=None):
        # Counter publisher
        self.count_publisher = rospy.Publisher("/rasberry_perception/tracking/unique_count", String, queue_size=1)

        # Subscribers
        self.labels = set()
        if people_tracker_topic is None:
            people_tracker_topic = '/rasberry_perception/tracking/results_array'
        self.tracker_results_subscriber = rospy.Subscriber(people_tracker_topic, TrackerResults, self._track_callback)

        if reset_topic is not None:
            self.reset_on = rospy.Subscriber(reset_topic, String, callback=self._reset)

    def _reset(self, *args):
        self.labels.clear()
        self.count_publisher.publish(String(str(len(self.labels))))

    def _track_callback(self, tracker_results_msg):
        for track in tracker_results_msg.tracks:
            self.labels.add(track.id)
        self.count_publisher.publish(String(str(len(self.labels))))
        print(str(len(self.labels)))


def __count_unique_tracks():
    rospy.init_node('rasberry_tracking_counter', anonymous=True)
    tracker_topic = rospy.get_param("~tracker_topic", None)
    reset_topic = rospy.get_param("~reset_topic", "/sequence_0/info")
    counter = CountAnnouncer(tracker_topic, reset_topic)
    rospy.spin()


if __name__ == '__main__':
    __count_unique_tracks()
