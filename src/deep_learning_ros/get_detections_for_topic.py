#!/usr/bin/env python
from __future__ import absolute_import, division, print_function

import message_filters
import numpy as np
import ros_numpy
import rospy
import cv2
import tf
from geometry_msgs.msg import PointStamped, Point
from rasberry_perception.msg import ImageDetections, HarvestDetections, SegmentationLabel3D, BoundingBox3D
from sensor_msgs.msg import Image, CameraInfo

from deep_learning_ros.compatibility_layer.detection_server import DetectorResultsClient, DETECTOR_OK
from linear_3dof_arm.control import Linear3dofController
from rasberry_perception_pkg.utility import function_timer
from rasberry_perception_pkg.visualisation import draw_detection_msg_on_image, PoseArrayPublisher


class DeepLearningRosInference:
    def __init__(self, colour_ns, depth_ns, score_thresh=0.5):
        self.controller = Linear3dofController()
        self.colour_topic = colour_ns + "/image_raw"
        self.colour_info_topic = colour_ns + "/camera_info"
        self.depth_topic = depth_ns + "/image_raw"
        self.depth_info_topic = depth_ns + "/camera_info"

        self.visualisation_topic = "/detection/image_raw"
        self.detections_topic = "/detection/predictions"
        self.harvest_detections_topic = "/detection/predictions_points"
        self.score_thresh = score_thresh
        self.std_thresh = 0.5  # Within 68% of the data

        # Wait for connection to detection service
        self.detector = DetectorResultsClient()

        # Initialise publisher
        self.detection_vis_pub = rospy.Publisher(self.visualisation_topic, Image, queue_size=1)
        self.detections_pub = rospy.Publisher(self.detections_topic, ImageDetections, queue_size=1)
        self.harvest_detections_pub = rospy.Publisher(self.harvest_detections_topic, HarvestDetections, queue_size=1)

        self.tf_listener = tf.TransformListener()
        self.parent_frame = "linear_3dof_arm_home"
        self.pose_array_image_publisher = PoseArrayPublisher("/detection/image_pose_array")
        self.pose_array_arm_publisher = PoseArrayPublisher("/detection/arm_pose_array", frame_id=self.parent_frame)

        # Initialise subscribers
        self.colour_sub = message_filters.Subscriber(self.colour_topic, Image)
        self.depth_sub = message_filters.Subscriber(self.depth_topic, Image)
        self.depth_info_sub = message_filters.Subscriber(self.depth_info_topic, CameraInfo)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.colour_sub, self.depth_sub, self.depth_info_sub], 10, 0.5)

        self.ts.registerCallback(self.run_detector)

    @function_timer.interval_logger(interval=10)
    def run_detector(self, colour_msg, depth_msg, depth_info_msg):
        # position = self.controller.current_position
        # if abs(position.x - 500) > 1 or abs(position.y) > 1 or abs(position.z - 500) > 1:
        #     print("not running detection (not in position)", abs(position.x - 500), abs(position.y - 0), abs(position.z - 500))
        #     return

        result = self.detector(image=colour_msg, score_thresh=self.score_thresh)
        if result.status != DETECTOR_OK:
            return

        self.detections_pub.publish(result.detections)

        rgb_image = ros_numpy.numpify(colour_msg)
        depth_image = ros_numpy.numpify(depth_msg)

        # If mask information available then publish 3D information too
        if len(result.detections.instances):
            fx, fy, cx, cy = depth_info_msg.P[0], depth_info_msg.P[5], depth_info_msg.P[2], depth_info_msg.P[6]
            bounding_boxes_ = []
            segmentation_labels_ = []

            image_frame_pose_array_points = []
            arm_frame_pose_array_points = []

            for instance in result.detections.instances:
                # Get roi depth points
                x_l = np.asarray(instance.x)
                y_l = np.asarray(instance.y)
                z_l = depth_image[instance.x, instance.y]
                # Filter the points
                valid_ind = np.where(z_l != 0)
                x_l = x_l[valid_ind]
                y_l = y_l[valid_ind]
                z_l = z_l[valid_ind]

                # Filter out by std_dev (remove outliers)
                valid_ind = abs(z_l - np.mean(z_l)) < self.std_thresh * np.std(z_l)
                x_l = x_l[valid_ind]
                y_l = y_l[valid_ind]
                z_l = z_l[valid_ind]

                # Project points to 3D space
                x = ((x_l - cx) * z_l / fx) / 1000.0
                y = ((y_l - cy) * z_l / fy) / 1000.0
                z = z_l / 1000.0

                # Get detection point in image space
                image_frame_pose_array_points.append([np.average(x), np.average(y), np.average(z)])

                if len(x) == 0:
                    continue
                if not (len(x) == len(y) == len(z)):
                    raise ValueError("Array dimensions do not agree x={}, y={} and z={}".format(len(x), len(y), len(z)))
                segmentation_labels_.append(SegmentationLabel3D(x=x, y=y, z=z, class_id=instance.class_id))

                xa, ya, za, x_r, y_r = np.average(x), np.average(y), np.average(z), np.ptp(x) / 2, np.ptp(y) / 2
                wp = PointStamped(header=colour_msg.header, point=Point(xa, ya, za))
                try:
                    wp = self.tf_listener.transformPoint(self.parent_frame, wp)
                except tf.ExtrapolationException as e:
                    print("\tSkipping {}, {}, {} due to exception '{}'".format(xa, ya, za, e))
                    continue
                xa, ya, za = wp.point.x, wp.point.y, wp.point.z
                arm_frame_pose_array_points.append([xa, ya, za])
                bounding_boxes_.append(BoundingBox3D(x=xa, y=ya, z=za, height_radius=x_r, width_radius=y_r))

            self.pose_array_image_publisher.visualise_points(image_frame_pose_array_points, header=depth_msg.header)
            self.pose_array_arm_publisher.visualise_points(arm_frame_pose_array_points)
            self.harvest_detections_pub.publish(HarvestDetections(header=colour_msg.header,
                                                                  bounding_boxes=bounding_boxes_,
                                                                  instances=segmentation_labels_,
                                                                  class_labels=result.detections.class_labels))

        # Publish detection visualisation topic
        vis_canvas = rgb_image
        vis_canvas = draw_detection_msg_on_image(vis_canvas, result.detections, encoding=colour_msg.encoding)
        detection_visualisation_msg = ros_numpy.msgify(Image, vis_canvas, encoding=colour_msg.encoding)
        detection_visualisation_msg.header = colour_msg.header
        self.detection_vis_pub.publish(detection_visualisation_msg)


def _get_detections_for_topic():
    rospy.init_node('deep_learning_detector', anonymous=True)

    # get private namespace parameters
    # p_image_ns = rospy.get_param('~image_ns', "/pico_zense/colour")
    # p_depth_ns = rospy.get_param('~depth_ns', "/pico_zense/aligned_depth_to_colour")
    p_image_ns = rospy.get_param('~image_ns', "/linear_3dof_2d_camera/color")
    p_depth_ns = rospy.get_param('~depth_ns', "/linear_3dof_2d_camera/aligned_depth_to_color")
    p_score = rospy.get_param('~score', 0.5)

    rospy.loginfo("Camera Topic to Detection ROS: ")

    detector = DeepLearningRosInference(colour_ns=p_image_ns, depth_ns=p_depth_ns, score_thresh=p_score)
    rospy.spin()


if __name__ == '__main__':
    _get_detections_for_topic()
