#!/usr/bin/env python
from __future__ import absolute_import, division, print_function

from copy import deepcopy

import message_filters
import ros_numpy
import rospy
from geometry_msgs.msg import PoseArray, Pose, Quaternion, Point
from sensor_msgs.msg import Image, CameraInfo
from rasberry_perception.msg import LabelledImage

from rasberry_perception_pkg.detection import DetectorClient, BBoxProjector
from rasberry_perception_pkg.visualisation import MarkerPublisher


class StrawberryDetector:
    def __init__(self, camera_name, host, model_name, score_thresh, method, distance_thresh, method_thresh, plot):
        self.camera_name = camera_name
        self.detector_host = host
        self.detector_model_name = model_name
        self.score_thresh = score_thresh
        self.point_method = method
        self.point_distance_thresh = distance_thresh
        self.point_method_thresh = method_thresh
        self.plot = plot

        self.parent_frame_id = '/linear_3dof_arm_home'
        self.child_frame_id = '/{}_color_optical_frame'.format(self.camera_name)

        # Define 2D detector
        self.detector_2D = DetectorClient(host=self.detector_host, model_name=self.detector_model_name)
        self.bbox_projector_to_3D = BBoxProjector(method=self.point_method, distance_thresh=self.point_distance_thresh,
                                                  method_thresh=self.point_method_thresh, parent=self.parent_frame_id,
                                                  child=self.child_frame_id)

        base = "/" + self.camera_name
        # Visualisation publishers
        if self.plot:
            self.marker_publisher = MarkerPublisher(base + "/sync/aligned_depth/picking_markers", self.parent_frame_id,
                                                    self.detector_2D.COLOURS)
            self.detection_summary = rospy.Publisher(base + "/sync/color/detection_summary", Image, queue_size=1)
            self.detection_map_pub = rospy.Publisher(base + "/color/detection_map_image_raw", Image, queue_size=1)

        # Republish depth map with synchronised timestamp
        # TODO: Remove this in favour of labeled_depth publisher
        self.depth_map_pub = rospy.Publisher(base + "/sync/aligned_depth/image_raw", Image, queue_size=1)
        self.depth_info_pub = rospy.Publisher(base + "/sync/aligned_depth/camera_info", CameraInfo, queue_size=1)

        self.labelled_depth_pub = rospy.Publisher('/rasberry_perception/detection/depth_bbox', LabelledImage, queue_size=1)
        self.labelled_rgb_pub = rospy.Publisher('/rasberry_perception/detection/rgb_bbox', LabelledImage, queue_size=1)
        self.pose_array_pub = rospy.Publisher('/rasberry_perception/detection/pose_array', PoseArray, queue_size=1)

        # Configure subscribers
        self.colour_sub = message_filters.Subscriber(base + "/color/image_raw", Image)
        self.depth_sub = message_filters.Subscriber(base + "/aligned_depth_to_color/image_raw", Image)
        self.depth_info_sub = message_filters.Subscriber(base + "/aligned_depth_to_color/camera_info", CameraInfo)

        self.ts = message_filters.ApproximateTimeSynchronizer([self.colour_sub, self.depth_sub, self.depth_info_sub],
                                                              10, 0.5)
        self.busy = False
        self.ts.registerCallback(self.detect_wrapper)
        self.time_between_frames, self.last_detect_time = 0, rospy.Time.now()

    def detect_wrapper(self, rgb_message, depth_message, depth_info_message):
        if self.busy:
            return
        self.busy = True

        now = rospy.Time.now()
        self.time_between_frames = now - self.last_detect_time
        self.last_detect_time = now

        self.detect(rgb_message, depth_message, depth_info_message)

        self.busy = False

    def detect(self, rgb_message, depth_message, depth_info_message):
        sync_header = rgb_message.header
        child_sync_header = deepcopy(sync_header)
        child_sync_header.frame_id = self.child_frame_id
        parent_sync_header = deepcopy(sync_header)
        parent_sync_header.frame_id = self.parent_frame_id

        # Get raw image data
        rgb_image = ros_numpy.numpify(rgb_message)
        depth_image = ros_numpy.numpify(depth_message)

        # Get location of objects
        bounding_boxes, masks, valid_class_labels = self.detector_2D.get_bounding_boxes(rgb_image[..., ::-1],
                                                                                        score=self.score_thresh)
        if bounding_boxes is None:
            return

        # Create copies of depth maps for synchronisation and later use
        depth_map_msg = depth_message
        depth_map_msg.header = sync_header
        depth_info_msg = depth_info_message
        depth_info_msg.header = sync_header

        # Get projected points from bounding box
        object_annotations, valid_bounding_boxes = self.bbox_projector_to_3D.bbox_to_object_annotation(
            depth_image, bounding_boxes, depth_info_msg.P, child_sync_header
        )

        # Visualise points
        if self.plot:
            # Get detection map (map of points used for object description calculation)
            detection_summary_image, detection_map = BBoxProjector.visualise_detection(rgb_image, valid_bounding_boxes,
                                                                                       self.detector_2D.CLASSES,
                                                                                       self.detector_2D.COLOURS)
            detection_summary_msg = ros_numpy.msgify(Image, detection_summary_image, encoding="rgb8")
            detection_summary_msg.header = child_sync_header
            self.detection_summary.publish(detection_summary_msg)

            centroids = [[getattr(d.description, s) for s in ['x', 'y', 'z', 'class_id']] for d in object_annotations]
            self.marker_publisher.visualise_points(centroids, child_sync_header)

            detection_map_msg = ros_numpy.msgify(Image, detection_map, encoding="rgb8")
            detection_map_msg.header = sync_header
            self.detection_map_pub.publish(detection_map_msg)

        # Publish detection messages
        labelled_message = LabelledImage(header=sync_header, parent_frame_id=self.parent_frame_id,
                                         image=rgb_message, annotations=object_annotations,
                                         class_labels=valid_class_labels)

        self.labelled_rgb_pub.publish(labelled_message)

        # Publish depth version for later analysis of depth map
        labelled_message.image = depth_map_msg
        self.labelled_depth_pub.publish(labelled_message)

        # Publish object description as PoseArray message
        poses = [Pose(position=Point(d.description.x, d.description.y, d.description.z), orientation=Quaternion(w=1))
                 for d in object_annotations]
        pose_array_msg = PoseArray(header=parent_sync_header, poses=poses)
        self.pose_array_pub.publish(pose_array_msg)

        # Synchronised messages
        self.depth_map_pub.publish(depth_map_msg)
        self.depth_info_pub.publish(depth_info_msg)


def detector_2d():
    rospy.init_node('rasberry_perception_detector_2d', anonymous=True)

    # get private namespace parameters
    p_camera_name = rospy.get_param('~camera_name', "linear_3dof_2d_camera1")
    p_host = rospy.get_param('~host', "0.0.0.0")
    p_model_name = rospy.get_param('~model_name', "faster-rcnn")
    p_score_thresh = rospy.get_param('~score_thresh', 0.5)

    p_selection_method = rospy.get_param('~point_selection_method', "rect")
    p_selection_distance_thresh = float(rospy.get_param('~point_selection_distance_thresh', 1.0))
    p_selection_method_thresh = float(rospy.get_param('~point_selection_method_thresh', 1.5))
    p_plot = rospy.get_param('~plot_picking_points', True)

    rospy.loginfo("Detector 2D: Initialising detector with camera: '{}', host '{}', model '{}' and score thresh: '{}', "
                  "point_selection_method='{}', point_selection_distance_thresh='{}m' and point_selection_method_thresh"
                  "='{}'".format(p_camera_name, p_host, p_model_name, p_score_thresh, p_selection_method,
                                 p_selection_distance_thresh, p_selection_method_thresh))

    detector = StrawberryDetector(camera_name=p_camera_name, host=p_host, model_name=p_model_name,
                                  score_thresh=p_score_thresh, method=p_selection_method,
                                  distance_thresh=p_selection_distance_thresh, method_thresh=p_selection_method_thresh,
                                  plot=p_plot)
    rospy.spin()


if __name__ == '__main__':
    detector_2d()
