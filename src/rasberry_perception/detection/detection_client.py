#!/usr/bin/env python

#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

# Executable for sending an image topic to the DetectionServer and publishing the results

from __future__ import absolute_import, division, print_function

from collections import defaultdict

import message_filters
import numpy as np
import ros_numpy
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, PoseArray
from sensor_msgs.msg import Image, CameraInfo

from rasberry_perception.detection import Client, default_service_name
from rasberry_perception.detection.utility import function_timer, WorkerTaskQueue
from rasberry_perception.detection.visualisation import Visualiser
from rasberry_perception.msg import Detections, Detection, RegionOfInterest, SegmentOfInterest, TaggedPoseStampedArray,\
    TaggedPose


class RunClientOnTopic:
    def __init__(self, image_namespace, depth_namespace=None, score_thresh=0.5, service_name=default_service_name,
                 visualisation_enabled=False):
        # Initialise class members
        self.score_thresh = score_thresh
        self._service_name = service_name
        self.namespace = "rasberry_perception/"
        self.depth_enabled = depth_namespace is not None
        self.visualisation_enabled = visualisation_enabled

        # Wait for connection to detection service
        self.detector = Client()

        # Initialise colour publishers/subscribers
        # These topics will republish the colour image (to ensure a 1:1 detection/source lookup)
        self.image_pub = rospy.Publisher(self.namespace + "colour/image_raw", Image, queue_size=1)
        self.image_info_pub = rospy.Publisher(self.namespace + "colour/camera_info", CameraInfo, queue_size=1)

        subscribers = [
            message_filters.Subscriber(image_namespace + "/image_raw", Image),
            message_filters.Subscriber(image_namespace + "/camera_info", CameraInfo),
        ]

        # The detector results publisher
        self.detection_results_pub = rospy.Publisher(self.namespace + "results", Detections, queue_size=1)

        # Initialise depth publishers/subscribers
        if self.depth_enabled:
            # These topics will republish the depth image (to ensure a 1:1 detection/source lookup)
            self.depth_pub = rospy.Publisher(self.namespace + "depth/image_raw", Image, queue_size=1)
            self.depth_info_pub = rospy.Publisher(self.namespace + "depth/camera_info", CameraInfo, queue_size=1)

            # Container for /detection/<class name>/bbox_poses and /detection/<class name>/segm_poses messages
            self.depth_pose_publishers = {}

            # Publish as tagged (by class name)
            self.tagged_bbox_pose_publisher = rospy.Publisher(self.namespace + "poses/bbox",
                                                              TaggedPoseStampedArray, queue_size=1)
            self.tagged_segm_pose_publisher = rospy.Publisher(self.namespace + "poses/segm",
                                                              TaggedPoseStampedArray, queue_size=1)

            # Subscribe to depth and depth intrinsic topics
            subscribers.extend([
                message_filters.Subscriber(depth_namespace + "/image_raw", Image),
                message_filters.Subscriber(depth_namespace + "/camera_info", CameraInfo),
            ])

        if self.visualisation_enabled:
            # Draw the detection message visually
            self.detections_vis_pub = rospy.Publisher(self.namespace + "vis/detection/image_raw", Image, queue_size=1)

            # Worker thread to do the heavy lifting of the detections visualisation
            self.publisher_tasks = WorkerTaskQueue(num_workers=1, max_size=1, discard=True)
            rospy.on_shutdown(self.on_shutdown)

            # This topic only publishes the points of the depth map that are considered for the bbox_pose calculation
            if self.depth_enabled:
                self.box_depth_pub = rospy.Publisher(self.namespace + "vis/bbox_points/image_raw", Image, queue_size=1)
                self.box_depth_info_pub = rospy.Publisher(self.namespace + "vis/bbox_points/camera_info", CameraInfo, queue_size=1)

        # Start subscription to the relevant topics
        sync_queue, sync_thresh = 1, 0.1
        rospy.loginfo("Waiting for topics with time synchroniser (queue {}, {}s tolerance) on '{}'".format(
            sync_queue, sync_thresh, ', '.join([s.topic for s in subscribers])
        ))
        self.ts = message_filters.ApproximateTimeSynchronizer(subscribers, sync_queue, sync_thresh, allow_headerless=True)
        self.ts.registerCallback(self.run_detector)

    def on_shutdown(self):
        self.publisher_tasks.stop()
        self.publisher_tasks.join()

    @function_timer.interval_logger(interval=10)
    def run_detector(self, *args, **kwargs):
        assert len(args) in [2, 4], "Args must either be (colour, info), or (colour, info, depth, info)"
        image_msg, image_info = args[:2]
        result = self.detector(image=image_msg)
        if not result.status.OKAY:
            return

        self.publish_detections(*args, response=result)

    def _publish_poses(self, poses, tagged_bbox_poses, tagged_segm_poses):
        """Creates a separate PoseArray publisher for each detected class and pose_origin and as a pose tagged

        Args:
            poses (Dict[str: PoseArray]): [Class Name][Origin] to PoseArray lookup of poses
        """
        if not self.depth_enabled:
            return

        if len(tagged_bbox_poses.poses):
            self.tagged_bbox_pose_publisher.publish(tagged_bbox_poses)
        if len(tagged_segm_poses.poses):
            self.tagged_segm_pose_publisher.publish(tagged_segm_poses)

        for class_name, origin_dict in poses.items():
            for origin, pose_array in origin_dict.items():
                topic = self.namespace + "poses/by_class/{}/{}".format(class_name.replace(" ", "_"), origin)
                if topic not in self.depth_pose_publishers:
                    self.depth_pose_publishers[topic] = rospy.Publisher(topic, PoseArray, queue_size=1)
                self.depth_pose_publishers[topic].publish(pose_array)

    @staticmethod
    def _get_pose(depth_roi, valid_positions, x_offset, y_offset, _fx, _fy, _cx, _cy):
        """Utility function to get a pose from a set of (y, x) points within a depth map"""
        zp = depth_roi[valid_positions] / 1000.0
        yp = ((valid_positions[0] + y_offset) - _cy) * zp / _fy
        xp = ((valid_positions[1] + x_offset) - _cx) * zp / _fx
        return Pose(position=Point(np.median(xp), np.median(yp), np.median(zp)), orientation=Quaternion(0, 0, 0, 1))

    @staticmethod
    def __check_pose_empty(p):
        return all([getattr(o, a, 0) == 0 for a in ["x", "y", "z", "w"] for o in [p.position, p.orientation]])

    @function_timer.interval_logger(interval=10)
    def publish_detections(self, image_msg, image_info, depth_msg, depth_info, response):
        """Function to publish detections based on the image data and detector result

        Args:
            image_msg (Image): The RGB image message
            image_info (CameraInfo): The RGB camera info message
            depth_msg (Image): The aligned depth to image_msg message
            depth_info (CameraInfo):  The depth camera info message
            response (GetDetectorResultsResponse):  The result of a call to the GetDetectorResults service api
        """
        # Filter detections by the score
        results = response.results
        results.objects = [d for d in response.results.objects if d.confidence >= self.score_thresh]
        results.camera_frame = image_msg  # TODO: Is it safe to assume the detection server won't add weird things here?
        results.camera_info = image_info
        # TODO: Is it safe to assume if all tracks have id==0 then they're not tracks?
        if len(results.objects) > 1 and all([d.track_id == 0 for d in results.objects]):
            for o in results.objects:
                o.track_id = -1  # Signify this is not a tracked object

        # Uncomment this line for dummy detections
        # results.objects.extend(self._get_test_messages(depth_msg.height, depth_msg.width))

        if not len(results.objects):
            return

        if self.depth_enabled and depth_msg is not None:
            depth_image = ros_numpy.numpify(depth_msg)
            boxes_depth_image = np.zeros_like(depth_image)

            fx, fy, cx, cy = depth_info.P[0], depth_info.P[5], depth_info.P[2], depth_info.P[6]

            # Publish in the frame of depth_msg.header.frame_id
            # [Class Name][Origin] to PoseArray lookup
            poses_header = depth_msg.header
            poses = defaultdict(lambda: defaultdict(lambda: PoseArray(header=poses_header)))
            tagged_bbox_poses = TaggedPoseStampedArray(header=poses_header)
            tagged_segm_poses = TaggedPoseStampedArray(header=poses_header)
            tagged_real_poses = PoseArray(header=poses_header)

            for i in range(len(results.objects)):
                label = results.objects[i].class_name

                # Get localisation from bbox
                roi = results.objects[i].roi
                xv, yv = np.meshgrid(np.arange(int(roi.x1), int(roi.x2)), np.arange(int(roi.y1), int(roi.y2)))
                d_roi = depth_image[yv, xv]  # For bbox the x, y, z pos is based on median of valid depth pixels
                valid_idx = np.where(np.logical_and(d_roi != 0, np.isfinite(d_roi)))

                infer_pose_from_depth = self.__check_pose_empty(results.objects[i].pose)
                if len(valid_idx[0]) and len(valid_idx[1]):
                    box_pose = results.objects[i].pose
                    if infer_pose_from_depth:
                        box_pose = self._get_pose(d_roi, valid_idx, roi.x1, roi.y1, fx, fy, cx, cy)
                        results.objects[i].pose = box_pose
                        results.objects[i].pose_frame_id = depth_msg.header.frame_id
                    tagged_bbox_poses.poses.append(TaggedPose(tag=label, pose=box_pose))
                    poses[label]["bbox"].poses.append(box_pose)
                if self.visualisation_enabled:
                    boxes_depth_image[yv, xv] = depth_image[yv, xv]  # Publish a bbox pixels only image

                # Get localisation from segm
                segm = results.objects[i].seg_roi

                if not (segm.x and segm.y):
                    continue

                d_roi = depth_image[segm.y, segm.x]  # For segm the x,y,z pos is based on median of detected pixels
                valid_idx = np.where(np.logical_and(d_roi != 0, np.isfinite(d_roi)))
                if len(valid_idx[0]):
                    zp = d_roi[valid_idx[0]] / 1000.0
                    yp = (np.asarray(segm.y)[valid_idx[0]] - cy) * zp / fy
                    xp = (np.asarray(segm.x)[valid_idx[0]] - cx) * zp / fx
                    segm_pose = Pose(position=Point(np.median(xp), np.median(yp), np.median(zp)),
                                     orientation=Quaternion(0, 0, 0, 1))
                    tagged_segm_poses.poses.append(TaggedPose(tag=label, pose=segm_pose))
                    poses[label]["segm"].poses.append(segm_pose)
                    if infer_pose_from_depth:
                        results.objects[i].pose = segm_pose  # should be more accurate so use this as default

            # Publish depth poses and 1:1 depth map
            self._publish_poses(poses, tagged_bbox_poses, tagged_segm_poses)
            self.depth_pub.publish(depth_msg)
            self.depth_info_pub.publish(depth_info)

            if self.visualisation_enabled:
                # Publish bbox points only depth map
                box_depth_msg = ros_numpy.msgify(Image, boxes_depth_image, encoding="16UC1")
                box_depth_msg.header = depth_msg.header
                self.box_depth_pub.publish(box_depth_msg)
                self.box_depth_info_pub.publish(depth_info)

        # Publish detection results
        self.detection_results_pub.publish(results)

        # Republish colour images
        self.image_pub.publish(image_msg)
        self.image_info_pub.publish(image_info)

        # Offload the visualisation task <1ms (takes considerable time)
        if self.visualisation_enabled:
            self.publisher_tasks.add_task(self._vis_publish, (image_msg, results,))

    @function_timer.interval_logger(interval=10)
    def _vis_publish(self, image_msg, result):
        """Publish function for service results. Meant to be offloaded to another thread.

        Args:
            image_msg (Image): The RGB image message
            image_info (CameraInfo): The RGB camera info message
            depth_msg (Image): The aligned depth to image_msg message
            depth_info (CameraInfo):  The depth camera info message
            result (GetDetectorResultsResponse):  The result of a call to the GetDetectorResults service api
        """
        vis = Visualiser(ros_numpy.numpify(image_msg))
        vis.draw_detections_message(result)
        vis_image = vis.get_image(overlay_alpha=0.5)
        vis_msg = ros_numpy.msgify(Image, vis_image, encoding=image_msg.encoding)
        vis_msg.header = image_msg.header
        self.detections_vis_pub.publish(vis_msg)


    def _get_test_messages(self, max_y, max_x, n_grid=2, box_height=200, box_width=200):
        """Internal use only. Returns a list of fake detections

        Args:
            max_y (int): Max y position (height) of the bounding boxes
            max_x (int): Max x position (width) of the bounding boxes
        """
        detections = []
        margin = 20
        max_y -= box_height + (margin * 2)
        max_x -= box_width + (margin * 2)

        for i in range(n_grid + 1):
            y = max_y * (i / n_grid) + margin
            for j in range(n_grid + 1):
                x = max_x * (j / n_grid) + margin
                roi = RegionOfInterest(x1=x, x2=x + box_width, y1=y, y2=y + box_height)
                seg_roi = SegmentOfInterest(x=range(int(roi.x1), int(roi.x2)), y=range(int(roi.y1), int(roi.y2)))
                detections.append(Detection(confidence=np.random.uniform(), class_name="test", roi=roi, seg_roi=seg_roi))
        return detections


def _get_detections_for_topic():
    service_name = default_service_name
    _node_name = service_name + "_client"
    rospy.init_node(_node_name, anonymous=True)

    # get private namespace parameters
    p_image_ns = rospy.get_param('~image_ns', "/sequence_0/colour")
    p_depth_ns = rospy.get_param('~depth_ns', "/sequence_0/depth")
    p_score = rospy.get_param('~score', 0.5)
    p_vis = rospy.get_param('~show_vis', True)

    rospy.loginfo("Camera Topic to Detection ROS: image_namespace={}, depth_namespace={}, score_thresh={}".format(
        p_image_ns, p_depth_ns, p_score
    ))

    detector = RunClientOnTopic(image_namespace=p_image_ns, depth_namespace=p_depth_ns, score_thresh=p_score,
                                visualisation_enabled=p_vis, service_name=service_name)
    rospy.spin()


if __name__ == '__main__':
    _get_detections_for_topic()
