import rospy
from std_msgs.msg import Header
from visualization_msgs.msg import Marker


class MarkerPublisher:
    def __init__(self, name, frame_id="map", colours=None, queue_size=1):
        self.publisher = rospy.Publisher(name, Marker, queue_size=queue_size)
        self.frame_id = frame_id

        self.clear_marker = Marker(action=3)
        self.clear_marker.header.frame_id = self.frame_id
        self.clear_marker.header.stamp = rospy.Time()

        if colours is None:
            colours = [[1.0, 1.0, 1.0]]
        if max(colours[0]) > 1:
            colours = [[c2 / 255.0 for c2 in c1] for c1 in colours]
        self.colours = colours

        self.__last_object_id = -1

    def clear_markers(self):
        self.publisher.publish(self.clear_marker)
        self.__last_object_id = -1

    def get_object_id(self):
        self.__last_object_id += 1
        return self.__last_object_id

    def visualise_points(self, point_information, header=None, clear=True, duration=None):
        # Create visualisation for markers
        if clear:
            # Clear previous markers
            self.clear_markers()

        # Fill marker array
        if header is None:
            header = Header()
            header.stamp = rospy.Time.now()

        for point_info in point_information:
            if len(point_info) not in (3, 4):
                rospy.logerr("Point array '{}' needs to be [x, y, z] or [x, y, z, colour_index]".format(point_info))
                continue

            x, y, z = point_info[:3]
            colour_id = 0
            if len(point_info) == 4:
                colour_id = point_info[3]

            marker = Marker()
            marker.header = header
            marker.header.frame_id = self.frame_id
            marker.id = self.get_object_id()
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.a = 1.0
            colour = self.colours[colour_id]
            marker.color.r = colour[0]
            marker.color.g = colour[1]
            marker.color.b = colour[2]
            marker.lifetime = rospy.Duration() if duration is None else rospy.Duration(duration)
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z
            self.publisher.publish(marker)
