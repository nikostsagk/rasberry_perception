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

from rasberry_perception import Client, default_service_name
from rasberry_perception.utility import function_timer, WorkerTaskQueue
from rasberry_perception.visualisation import Visualiser
from rasberry_perception.msg import Detections, Detection, RegionOfInterest, SegmentOfInterest, TaggedPoseStampedArray,\
    TaggedPose, ObjectSize


class RunClientOnTopic:
    def __init__(self, image_namespace, depth_namespace=None, score_thresh=0.5, service_name=default_service_name,
                 visualisation_enabled=False, publish_source=False):
        # Initialise class members
        self.score_thresh = score_thresh
        self._service_name = service_name
        self.namespace = "rasberry_perception/"
        self.depth_enabled = bool(depth_namespace)
        self.visualisation_enabled = visualisation_enabled
        self.publish_source = publish_source

        # Wait for connection to detection service
        self.detector = Client()

        # Initialise colour publishers/subscribers
        if self.publish_source:
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
        self.all_pose_publishers = {}
        if self.depth_enabled:
            if self.publish_source:
                # These topics will republish the depth image (to ensure a 1:1 detection/source lookup)
                self.depth_pub = rospy.Publisher(self.namespace + "depth/image_raw", Image, queue_size=1)
                self.depth_info_pub = rospy.Publisher(self.namespace + "depth/camera_info", CameraInfo, queue_size=1)

            # Container for /detection/<class name>/bbox_poses and /detection/<class name>/segm_poses messages
            self.depth_pose_publishers = {}

            # Publish as tagged (by class name)
            self.tagged_bbox_pose_publisher = rospy.Publisher(self.namespace + "poses/tagged/bbox",
                                                              TaggedPoseStampedArray, queue_size=1)
            self.tagged_segm_pose_publisher = rospy.Publisher(self.namespace + "poses/tagged/segm",
                                                              TaggedPoseStampedArray, queue_size=1)

            # Subscribe to depth and depth intrinsic topics
            subscribers.extend([
                message_filters.Subscriber(depth_namespace + "/image_raw", Image),
                message_filters.Subscriber(depth_namespace + "/camera_info", CameraInfo),
            ])

        if self.visualisation_enabled:
            # Draw the detection message visually
            self.detections_vis_pub = rospy.Publisher(self.namespace + "vis/detection/image_raw", Image, queue_size=1)
            self.detections_vis_info_pub = rospy.Publisher(self.namespace + "vis/detection/camera_info", CameraInfo,
                                                           queue_size=1)

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

        all_poses = {}
        for class_name, origin_dict in poses.items():
            for origin, pose_array in origin_dict.items():
                topic = self.namespace + "poses/by_class/{}/{}".format(class_name.replace(" ", "_"), origin)
                if topic not in self.depth_pose_publishers:
                    self.depth_pose_publishers[topic] = rospy.Publisher(topic, PoseArray, queue_size=1)
                if pose_array.header.frame_id not in all_poses:
                    all_poses[pose_array.header.frame_id] = PoseArray(header=pose_array.header)
                all_poses[pose_array.header.frame_id].poses.extend(pose_array.poses)
                self.depth_pose_publishers[topic].publish(pose_array)
        for frame_id, pose_array in all_poses.items():
            if frame_id not in self.all_pose_publishers:
                self.all_pose_publishers[frame_id] = rospy.Publisher(self.namespace + "poses/all/{}".format(frame_id),
                                                                     PoseArray, queue_size=1)
            self.all_pose_publishers[frame_id].publish(pose_array)

    @staticmethod
    def _reject_outliers(data, m=3.5, method="mad"):
        if not data.size or not data.all():
            return data
        if method == "std":
            return data[abs(data - np.mean(data)) < m * np.std(data)]
        elif method == "mad":
            d = data
            if len(d.shape) == 1:
                d = d[:, None]
            median = np.median(d, axis=0)
            diff = np.sum((d - median) ** 2, axis=-1)
            diff = np.sqrt(diff)
            med_abs_deviation = np.median(diff)
            modified_z_score = 0.6745 * diff / med_abs_deviation
            return data[modified_z_score > m]
        elif method == "median":
            d = np.abs(data - np.median(data))
            m_dev = np.median(d)
            s = d / (m_dev if m_dev else 1.)
            return data[s < m]
        else:
            raise ValueError("Method must be std or median")

    @staticmethod
    def _get_size(xp, yp, zp, method="mad", m=3.5):
        """Utility function to get rough object size from depth points"""
        frustrum_points = np.asarray([xp, yp, zp]).swapaxes(0, 1)  # shape=(N observations, 3)
        # TODO: Median absolute difference outlier rejection doesn't work when all points are similar
        object_points = RunClientOnTopic._reject_outliers(frustrum_points, m=m, method=method)
        if object_points.shape[0] == 0:
            return ObjectSize(0, 0, 0)
        w, h, d = object_points.ptp(axis=0).tolist()
        return ObjectSize(w, h, d)

    @staticmethod
    def _get_pose(depth_roi, valid_positions, x_offset, y_offset, _fx, _fy, _cx, _cy, return_size=False):
        """Utility function to get a pose from a set of (y, x) points within a depth map"""
        zp = depth_roi[valid_positions] / 1000.0
        yp = ((valid_positions[0] + y_offset) - _cy) * zp / _fy
        xp = ((valid_positions[1] + x_offset) - _cx) * zp / _fx
        ret = Pose(position=Point(np.median(xp), np.median(yp), np.median(zp)), orientation=Quaternion(0, 0, 0, 1))
        if return_size:
            ret = (ret, RunClientOnTopic._get_size(xp, yp, zp))
        return ret

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
        if self.publish_source:
            # TODO: Is it safe to assume the detection server won't add weird things here?
            results.camera_frame = image_msg
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
                        box_pose, size = self._get_pose(d_roi, valid_idx, roi.x1, roi.y1, fx, fy, cx, cy, return_size=True)
                        results.objects[i].pose = box_pose
                        results.objects[i].size = size
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
                    # Overwrite the bbox extracted size if segm available
                    results.objects[i].size = RunClientOnTopic._get_size(xp, yp, zp)
                    tagged_segm_poses.poses.append(TaggedPose(tag=label, pose=segm_pose))
                    poses[label]["segm"].poses.append(segm_pose)
                    if infer_pose_from_depth:
                        results.objects[i].pose = segm_pose  # should be more accurate so use this as default

                poses[label]["pose"].poses.append(results.objects[i].pose)
            # Publish depth poses and 1:1 depth map
            self._publish_poses(poses, tagged_bbox_poses, tagged_segm_poses)
            if self.publish_source:
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
        if self.publish_source:
            self.image_pub.publish(image_msg)
            self.image_info_pub.publish(image_info)

        # Offload the visualisation task <1ms (takes considerable time)
        if self.visualisation_enabled:
            self.publisher_tasks.add_task(self._vis_publish, (image_msg, image_info, results,))

    @function_timer.interval_logger(interval=10)
    def _vis_publish(self, image_msg, image_info, result):
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
        vis_info = image_info
        vis_info.header = vis_msg.header
        self.detections_vis_pub.publish(vis_msg)
        self.detections_vis_info_pub.publish(vis_info)


def _get_detections_for_topic():
    service_name = default_service_name
    _node_name = service_name + "_client"
    rospy.init_node(_node_name, anonymous=True)

    # get private namespace parameters
    p_image_ns = rospy.get_param('~image_ns', "/sequence_0/colour")
    p_depth_ns = rospy.get_param('~depth_ns', "/sequence_0/depth")
    p_score = rospy.get_param('~score', 0.5)
    p_vis = rospy.get_param('~show_vis', False)
    p_source = rospy.get_param('~publish_source', False)

    rospy.loginfo("Camera Topic to Detection ROS: image_namespace={}, depth_namespace={}, score_thresh={}, "
                  "visualisation_enabled={}, publish_source={}".format(p_image_ns, p_depth_ns, p_score, p_vis,
                                                                       p_source))

    try:
        detector = RunClientOnTopic(image_namespace=p_image_ns, depth_namespace=p_depth_ns, score_thresh=p_score,
                                    visualisation_enabled=p_vis, service_name=service_name, publish_source=p_source)
        rospy.spin()
    except (KeyboardInterrupt, rospy.ROSInterruptException) as e:
        print("Exiting node due to interrupt:", e)


if __name__ == '__main__':
    _get_detections_for_topic()
