import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField


class PointCloud2Publisher:
    def __init__(self):
        self.publishers = {}

    @staticmethod
    def xyz_array_to_pc2(points, colours=None, stamp=None, frame_id=None, seq=None):
        '''
        Create a sensor_msgs.PointCloud2 from an array
        of points.
        '''
        msg = PointCloud2()

        if stamp:
            msg.header.stamp = stamp
        if frame_id:
            msg.header.frame_id = frame_id
        if seq:
            msg.header.seq = seq

        if len(points.shape) == 3:
            msg.height = points.shape[1]
            msg.width = points.shape[0]
        else:
            msg.height = 1
            msg.width = len(points)

        msg.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                      PointField('y', 4, PointField.FLOAT32, 1),
                      PointField('z', 8, PointField.FLOAT32, 1)]

        data = points
        point_step = 12

        if colours is not None:
            assert (points.shape == colours.shape)
            msg.fields += [PointField('r', 12, PointField.FLOAT32, 1),
                           PointField('g', 16, PointField.FLOAT32, 1),
                           PointField('b', 20, PointField.FLOAT32, 1)]
            data = np.hstack([points, colours])
            point_step = 24

        msg.is_bigendian = False
        msg.point_step = point_step
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = int(np.isfinite(points).all())
        msg.data = np.asarray(data, np.float32).tostring()

        return msg

    def publish(self, message, publisher="example_pc2"):
        if publisher not in self.publishers:
            self.publishers[publisher] = rospy.Publisher("/{}".format(publisher), PointCloud2, queue_size=1)

        self.publishers[publisher].publish(message)
