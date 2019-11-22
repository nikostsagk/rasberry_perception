#!/usr/bin/env python
from __future__ import absolute_import, division, print_function

import message_filters
import numpy as np
import ros_numpy
import rospy
import tf
from geometry_msgs.msg import PointStamped, Point
from rasberry_perception.msg import ImageDetections, HarvestDetections, SegmentationLabel3D, BoundingBox3D
from sensor_msgs.msg import Image, CameraInfo

from deep_learning_ros.compatibility_layer.detection_server import DetectorResultsClient, DETECTOR_OK
from rasberry_perception_pkg.utility import function_timer
from rasberry_perception_pkg.visualisation import draw_detection_msg_on_image, PoseArrayPublisher


class DeepLearningRosInference:
    def __init__(self, colour_ns, depth_ns, score_thresh=0.5):
        self.colour_topic = colour_ns + "/image_raw"
        self.colour_info_topic = colour_ns + "/camera_info"
        self.depth_topic = depth_ns + "/image_raw"
        self.depth_info_topic = depth_ns + "/camera_info"

        self.namespace = "rasberry_perception/"
        self.visualisation_topic = self.namespace + "vis/image_raw"
        self.detections_topic = self.namespace + "detections"
        self.detections_pose_topic = self.namespace + "pose_array"

        self.score_thresh = score_thresh

        # Wait for connection to detection service
        self.detector = DetectorResultsClient()

        # Initialise publisher
        self.detections_pub = rospy.Publisher(self.detections_topic, ImageDetections, queue_size=1)
        self.detection_vis_pub = rospy.Publisher(self.visualisation_topic, Image, queue_size=1)
        self.pose_array_image_publisher = PoseArrayPublisher(self.detections_pose_topic)

        # Initialise subscribers
        self.colour_sub = message_filters.Subscriber(self.colour_topic, Image)
        self.depth_sub = message_filters.Subscriber(self.depth_topic, Image)
        self.depth_info_sub = message_filters.Subscriber(self.depth_info_topic, CameraInfo)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.colour_sub, self.depth_sub, self.depth_info_sub], 10, 0.5)

        self.ts.registerCallback(self.run_detector)

    @function_timer.interval_logger(interval=10)
    def run_detector(self, colour_msg, depth_msg, depth_info_msg):
        result = self.detector(image=colour_msg, score_thresh=self.score_thresh)
        if result.status != DETECTOR_OK:
            return

        # Add depth information to result and publish
        result.detections.aligned_depth = depth_msg
        result.detections.aligned_depth_info = depth_info_msg
        self.detections_pub.publish(result.detections)

        rgb_image = ros_numpy.numpify(colour_msg)
        depth_image = ros_numpy.numpify(depth_msg)

        # If mask information available then publish pose array information
        if len(result.detections.instances):
            fx, fy, cx, cy = depth_info_msg.P[0], depth_info_msg.P[5], depth_info_msg.P[2], depth_info_msg.P[6]

            pose_array_points = []

            for instance in result.detections.instances:
                # Get roi depth points
                x_l = np.asarray(instance.x)
                y_l = np.asarray(instance.y)
                z_l = depth_image[instance.y, instance.x]

                # Filter the points
                valid_ind = np.where(z_l != 0)
                x_l = x_l[valid_ind]
                y_l = y_l[valid_ind]
                z_l = z_l[valid_ind]

                # Project points to 3D space
                x = ((x_l - cx) * z_l / fx) / 1000.0
                y = ((y_l - cy) * z_l / fy) / 1000.0
                z = z_l / 1000.0

                if len(x) == 0 or len(y) == 0 or len(z) == 0:
                    continue

                # Get detection point in image space
                p_xyz = [np.median(x), np.median(y), np.median(z)]
                if np.nan not in p_xyz:
                    pose_array_points.append(p_xyz)

            self.pose_array_image_publisher.visualise_points(pose_array_points, header=colour_msg.header)


        # Publish detection visualisation topic
        vis_canvas = draw_detection_msg_on_image(rgb_image, result.detections, encoding=colour_msg.encoding)
        detection_visualisation_msg = ros_numpy.msgify(Image, vis_canvas, encoding=colour_msg.encoding)
        detection_visualisation_msg.header = colour_msg.header
        self.detection_vis_pub.publish(detection_visualisation_msg)


def _get_detections_for_topic():
    rospy.init_node('deep_learning_detector', anonymous=True)

    # get private namespace parameters
    p_image_ns = rospy.get_param('~image_ns', "/pico_zense/colour")
    p_depth_ns = rospy.get_param('~depth_ns', "/pico_zense/aligned_depth_to_colour")
    # p_image_ns = rospy.get_param('~image_ns', "/linear_3dof_2d_camera/color")
    # p_depth_ns = rospy.get_param('~depth_ns', "/linear_3dof_2d_camera/aligned_depth_to_color")
    p_score = rospy.get_param('~score', 0.5)

    rospy.loginfo("Camera Topic to Detection ROS: ")

    detector = DeepLearningRosInference(colour_ns=p_image_ns, depth_ns=p_depth_ns, score_thresh=p_score)
    rospy.spin()


if __name__ == '__main__':
    _get_detections_for_topic()
