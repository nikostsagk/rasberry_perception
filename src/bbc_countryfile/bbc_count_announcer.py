#!/usr/bin/env python
from __future__ import absolute_import, division, print_function

import sys

# Import python3 packages after removing ros
from std_msgs.msg import String

__ros_cv2_fix = '/opt/ros/kinetic/lib/python2.7/dist-packages'
sys.path.remove(__ros_cv2_fix)

from threading import Lock

# Import python2/ros packages after adding ros
sys.path.append(__ros_cv2_fix)
import rospy
from geometry_msgs.msg import PoseArray, Pose, Quaternion, Point

# import the necessary packages
from scipy.spatial import distance as dist
from collections import OrderedDict
import numpy as np
from bayes_people_tracker.msg import PeopleTracker


class CountAnnouncer:
    def __init__(self):
        # Counter publisher
        self.tracking_announcement = rospy.Publisher("/announcement", String, queue_size=1)

        # Subscribers
        self.uuids = set()
        self.tracker_publishes = rospy.Subscriber('/tracker/positions', PeopleTracker, self.detect_callback)

    def detect_callback(self, detection_message):
        for uuid in detection_message.uuids:
            self.uuids.add(uuid)
        self.tracking_announcement.publish(
            String(data="<p><b>Ripe Strawberry Count (bayes):</b> {}</p>".format(len(self.uuids)))
        )
        print("Bayes count: ", len(self.uuids))


def count_announcer():
    rospy.init_node('bbc_countryfile_count_announcer', anonymous=True)
    counter = CountAnnouncer()
    rospy.spin()


if __name__ == '__main__':
    count_announcer()
