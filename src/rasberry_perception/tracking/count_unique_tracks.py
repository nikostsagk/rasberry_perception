#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

import rospy
from bayes_people_tracker.msg import PeopleTracker
from std_msgs.msg import String


class CountAnnouncer:
    def __init__(self, people_tracker_topic=None, reset_topic=None):
        # Counter publisher
        self.count_publisher = rospy.Publisher("/rasberry_perception/tracker/unique_count", String, queue_size=1)

        # Subscribers
        self.uuids = set()
        if people_tracker_topic is None:
            people_tracker_topic = 'rasberry_perception/tracker/positions'
        self.positions_publisher = rospy.Subscriber(people_tracker_topic, PeopleTracker, self._track_callback)

        if reset_topic is not None:
            self.reset_on = rospy.Subscriber(reset_topic, String, callback=self._reset)

    def _reset(self, *args):
        self.uuids.clear()
        self.count_publisher.publish(String(str(len(self.uuids))))

    def _track_callback(self, detection_message):
        for uuid in detection_message.uuids:
            self.uuids.add(uuid)
        self.count_publisher.publish(String(str(len(self.uuids))))


def __count_unique_tracks():
    rospy.init_node('bayes_people_track_counter', anonymous=True)
    tracker_topic = rospy.get_param("~tracker_topic", None)
    reset_topic = rospy.get_param("~reset_topic", "/sequence_0/info")
    counter = CountAnnouncer(tracker_topic, reset_topic)
    rospy.spin()


if __name__ == '__main__':
    __count_unique_tracks()
