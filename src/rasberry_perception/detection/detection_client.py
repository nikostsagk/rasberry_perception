#!/usr/bin/env python

#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

# Executable for sending an image topic to the DetectionServer and publishing the results

from __future__ import absolute_import, division, print_function

import message_filters
import numpy as np
import ros_numpy
import rospy
from geometry_msgs.msg import Point, PoseArray, Pose
from sensor_msgs.msg import Image, CameraInfo

from rasberry_perception.detection import Client, default_service_name
from rasberry_perception.detection.utility import function_timer, WorkerTaskQueue
from rasberry_perception.detection.visualisation import Visualiser
from rasberry_perception.msg import Detections, Detection, RegionOfInterest, SegmentOfInterest, DetectionInfo


class RunClientOnTopic:
    def __init__(self, image_namespace, depth_namespace=None, score_thresh=0.5, service_name=default_service_name,
            vis=True):
        self._node_name = service_name + "_client"
        rospy.init_node(self._node_name, anonymous=True)
        stem = image_namespace.split('/')[-1]

        self.namespace = "rasberry_perception/" + stem + "/"

        self.score_thresh = score_thresh

        # Wait for connection to detection service
        self.detector = Client()
        self.vis = vis
        if self.vis:
            self.publisher_tasks = WorkerTaskQueue(num_workers=1, max_size=60)
            rospy.on_shutdown(self.on_shutdown)

        # Initialise publishers
        self.detections_pub = rospy.Publisher(self.namespace + "detections", Detections, queue_size=1)
        self.image_pub = rospy.Publisher(self.namespace + "colour/image_raw", Image, queue_size=1)
        self.image_info_pub = rospy.Publisher(self.namespace + "colour/camera_info", CameraInfo, queue_size=1)
        self.detections_vis_pub = rospy.Publisher(self.namespace + "colour/vis_raw", Image, queue_size=1)

        # Initialise subscribers
        subscribers = [
            message_filters.Subscriber(image_namespace + "/image_raw", Image),
            message_filters.Subscriber(image_namespace + "/camera_info", CameraInfo),
        ]

        self.depth_enabled = depth_namespace is not None
        if self.depth_enabled:
            self.depth_pub = rospy.Publisher(self.namespace + "depth/image_raw", Image, queue_size=1)
            self.depth_info_pub = rospy.Publisher(self.namespace + "depth/camera_info", CameraInfo, queue_size=1)
            self.depth_bbox_detections_pub = rospy.Publisher(self.namespace + "bbox_detections", PoseArray,
                                                             queue_size=1)
            self.depth_segm_detections_pub = rospy.Publisher(self.namespace + "segm_detections", PoseArray,
                                                             queue_size=1)

            subscribers.extend([
                message_filters.Subscriber(depth_namespace + "/image_raw", Image),
                message_filters.Subscriber(depth_namespace + "/camera_info", CameraInfo),
            ])

        # Start subscription
        sync_queue, sync_thresh = 1, 0.1
        rospy.loginfo("Waiting for topics with time synchroniser (queue {}, {}s tolerance) on '{}'".format(
            sync_queue, sync_thresh, ', '.join([s.topic for s in subscribers])
        ))
        self.ts = message_filters.ApproximateTimeSynchronizer(subscribers, 1, 0.1)
        self.ts.registerCallback(self.run_detector)

    def on_shutdown(self):
        self.publisher_tasks.stop()
        self.publisher_tasks.join()

    @function_timer.interval_logger(interval=10)
    def run_detector(self, *args, **kwargs):
        assert len(args) in [2, 4], "Args must either be (colour, info), or (colour, info, depth, info)"
        image_msg, image_info = args[:2]
        result = self.detector(image=image_msg, score_thresh=self.score_thresh)
        if not result.status.OKAY:
            return

        self.publish_detections(*args, result=result)

    @function_timer.interval_logger(interval=10)
    def publish_detections(self, image_msg, image_info, depth_msg, depth_info, result):
        """Function to publish detections based on the image data and detector result

        Args:
            image_msg (Image): The RGB image message
            image_info (CameraInfo): The RGB camera info message
            depth_msg (Image): The aligned depth to image_msg message
            depth_info (CameraInfo):  The depth camera info message
            result (GetDetectorResultsResponse):  The result of a call to the GetDetectorResults service api
        """
        detections = result.detections.detections
        detections.extend(self._get_test_messages(depth_msg.height, depth_msg.width))

        if not len(detections):
            return

        if self.depth_enabled and depth_msg is not None:
            depth_image = ros_numpy.numpify(depth_msg)

            fx, fy, cx, cy = depth_info.P[0], depth_info.P[5], depth_info.P[2], depth_info.P[6]

            bbox_poses = PoseArray(header=depth_msg.header)  # TODO: Change frame_id and translate by tf
            segm_poses = PoseArray(header=depth_msg.header)  # TODO: Change frame_id and translate by tf

            def _get_pose(depth_roi, valid_positions, x_offset, y_offset, _fx, _fy, _cx, _cy):
                zp = depth_roi[valid_positions] / 1000.0
                yp = ((valid_positions[0] + y_offset) - _cy) * zp / _fy
                xp = ((valid_positions[1] + x_offset) - _cx) * zp / _fx
                return Pose(position=Point(np.median(xp), np.median(yp), np.median(zp)))

            for detection in detections:
                # TODO: Transform by TF and intrinsics
                # Get localisation from bbox
                bbox = detection.roi
                xv, yv = np.meshgrid(np.arange(int(bbox.x1), int(bbox.x2)), np.arange(int(bbox.y1), int(bbox.y2)))
                d_roi = depth_image[yv, xv]  # For bbox the x, y, z pos is based on median of valid depth pixels
                valid_idx = np.where(np.logical_and(d_roi != 0, np.isfinite(d_roi)))
                if len(valid_idx[0]) and len(valid_idx[1]):
                    bbox_poses.poses.append(_get_pose(d_roi, valid_idx, bbox.x1, bbox.y1, fx, fy, cx, cy))

                # Get localisation from segm
                segm = detection.seg_roi
                xv, yv = np.meshgrid(segm.x, segm.y)
                d_roi = depth_image[yv, xv]  # For segm the x,y,z pos is based on median of detected pixels
                valid_idx = np.where(np.logical_and(d_roi != 0, np.isfinite(d_roi)))
                if len(valid_idx[0]) and len(valid_idx[1]):
                    segm_poses.poses.append(_get_pose(d_roi, valid_idx, bbox.x1, bbox.y1, fx, fy, cx, cy))

            # Publish pose arrays
            self.depth_pub.publish(depth_msg)
            self.depth_info_pub.publish(depth_info)
            self.depth_bbox_detections_pub.publish(bbox_poses)
            self.depth_segm_detections_pub.publish(segm_poses)

        # Publish detection results
        self.detections_pub.publish(result.detections)
        self.image_pub.publish(image_msg)
        self.image_info_pub.publish(image_info)

        # Offload the visualisation task <1ms (takes considerable time)
        if self.vis:
            self.publisher_tasks.add_task(self._vis_publish, (image_msg, result,))

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
        from timeit import default_timer as timer
        s = timer()
        vis = Visualiser(ros_numpy.numpify(image_msg))
        vis.draw_detections_message(result)
        vis_image = vis.get_image(overlay_alpha=1.0)
        vis_msg = ros_numpy.msgify(Image, vis_image, encoding="bgr8")
        vis_msg.header = image_msg.header
        self.detections_vis_pub.publish(vis_msg)
        es = timer()
        # print(es - s)


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
                detections.append(
                    Detection(
                        info=DetectionInfo(score=np.random.uniform(), class_id=-1, class_name="test"),
                        roi=roi,
                        seg_roi=seg_roi
                    )
                )
        return detections


def _get_detections_for_topic():
    # get private namespace parameters
    p_image_ns = rospy.get_param('~image_ns', "/pico_zense/colour")
    p_depth_ns = rospy.get_param('~depth_ns', "/pico_zense/aligned_depth_to_colour")
    p_score = rospy.get_param('~score', 0.5)

    rospy.loginfo("Camera Topic to Detection ROS: image_namespace={}, depth_namespace={}, score_thresh={}".format(
        p_image_ns, p_depth_ns, p_score
    ))

    # TODO: Re-implement depth for now leave as None
    detector = RunClientOnTopic(image_namespace=p_image_ns, depth_namespace=p_depth_ns, score_thresh=p_score)
    rospy.spin()


if __name__ == '__main__':
    _get_detections_for_topic()
