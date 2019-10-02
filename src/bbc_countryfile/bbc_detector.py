#!/usr/bin/env python
from __future__ import absolute_import, division, print_function

import sys

# Import python3 packages after removing ros
__ros_cv2_fix = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if __ros_cv2_fix in sys.path:
    sys.path.remove(__ros_cv2_fix)
    import cv2
    import torch
    from mmdet.apis import inference_detector, init_detector, show_result
    from copy import deepcopy
    from timeit import default_timer as timer
    from collections import deque
    from threading import Event

    # Import python2/ros packages after adding ros
    sys.path.append(__ros_cv2_fix)

import ros_numpy
import rospy
from geometry_msgs.msg import PoseArray, Pose, Quaternion, Point
from sensor_msgs.msg import Image, CameraInfo
from yeet.msg import LabelledImage, BoundingBox
# from yeet_pkg.detection import DetectorClient, BBoxProjector
from rasberry_perception_pkg.visualisation import MarkerPublisher
from bbc_countryfile.models import model_lookup
from bbc_countryfile.trackers import CentroidTracker
from std_msgs.msg import String


class CounterBBC:
    def __init__(self, camera_name, model_name, score_thresh):
        self.target_class = "Ripe Strawberry"
        self.camera_name = camera_name
        self.model_name = model_name
        self.score_thresh = score_thresh

        # Define 2D detector
        self.namespace = "/" + self.camera_name

        # Visualisation publishers
        self.detection_summary = rospy.Publisher("/vis/image_raw", Image, queue_size=1)
        self.tracking_announcement = rospy.Publisher("/announcement", String, queue_size=1)
        self.pose_array_pub = rospy.Publisher('/detection/pose_array', PoseArray, queue_size=1)
        # self.bounding_boxes = rospy.Publisher('/detection/bounding_boxes/image_raw', Image, queue_size=1)

        # Setup model
        assert self.model_name in model_lookup
        self.model_config = model_lookup[self.model_name]["config"]
        self.model_path = model_lookup[self.model_name]["model"]
        self.model = init_detector(self.model_config, self.model_path, device=torch.device('cuda', 0))
        self.frame_times = deque(maxlen=30)

        # Subscribers
        self.busy = Event()
        self.last_time = rospy.Time(0)  # ensure each subscriber callback is in the future
        self.color_sub = rospy.Subscriber(self.namespace + "/color/image_raw", Image, self.detect_callback,
                                          queue_size=1)
        self.centroid_tracker = CentroidTracker(max_disappeared=1)
        # self.stupid = 0

    def detect_callback(self, rgb_message):
        if self.busy.is_set() or rgb_message.header.stamp < self.last_time:
            return
        self.busy.set()
        start_time = timer()

        self.last_time = rgb_message.header.stamp
        self.detect(rgb_message)

        self.frame_times.append(timer() - start_time)
        average_time = sum(self.frame_times) / len(self.frame_times)
        print("{:.2f}ms FPS: {:.2f}".format(average_time * 1000, 1 / average_time))
        self.busy.clear()

    def detect(self, rgb_message):
        # Get raw image data
        rgb_image = ros_numpy.numpify(rgb_message)

        # Run the Model
        result = inference_detector(self.model, rgb_image)

        # Clean bounding boxes to Class Name: [[x1, y1, x2, y2, score],...]
        bounding_boxes = {self.model.CLASSES[i]: [list(bb) + [i] for bb in b if bb[-1] >= self.score_thresh] for i, b in
                          enumerate(result) if b.size}

        target_bounding_boxes = bounding_boxes[self.target_class] if self.target_class in bounding_boxes else []

        # Update very naive tracker
        self.centroid_tracker.update(target_bounding_boxes)
        self.tracking_announcement.publish(
            String(
                data="<p><b>Naive {} Count:</b> {}</p>".format(self.target_class, self.centroid_tracker.total_tracks()))
        )

        # Normalise to metric space (assuming 1.1m width viewport to row)
        x_scale = 1.1
        y_scale = (720 / 1280) * x_scale
        target_bounding_boxes = [
            [(t[0] / 1280) * x_scale, (t[1] / 720) * y_scale, (t[2] / 1280) * x_scale, (t[3] / 720) * y_scale, t[4],
             t[5]] for t in target_bounding_boxes]

        # Publish tracking information (centroids)
        ripe_centroids = [((b[0] + b[2]) / 2, (b[1] + b[3]) / 2) for b in target_bounding_boxes]
        poses = [Pose(position=Point(x=c[0], y=c[1]), orientation=Quaternion(w=1)) for c in ripe_centroids]
        pose_array = PoseArray(poses=poses)
        pose_array.header.frame_id = rgb_message.header.frame_id
        pose_array.header.stamp = rgb_message.header.stamp
        self.pose_array_pub.publish(pose_array)

        # Visualise
        self.visualise(rgb_image, result, centroids=self.centroid_tracker.objects.items())

    def visualise(self, rgb_image, result, centroids=None):
        visualisation_image = show_result(rgb_image, result, self.model.CLASSES, score_thr=self.score_thresh,
                                          wait_time=1, show=False)
        if centroids:
            # loop over the tracked objects
            for (object_id, centroid) in centroids:
                # draw both the ID of the object and the centroid of the
                # object on the output frame
                text = "ID: {}".format(object_id)
                cx, cy = centroid[0] - 10, centroid[1] - 10
                if 0 <= cx <= visualisation_image.shape[0] and 0 <= cy <= visualisation_image.shape[1]:
                    cv2.putText(visualisation_image, text, (cx, cy), cv2.FONT_HERSHEY_SIMPLEX,
                                0.8, (255, 50, 50), 2)
                    cv2.circle(visualisation_image, (centroid[0], centroid[1]), 4, (555, 50, 50), -1)

        # cv2.imwrite('imgs/{}.png'.format(self.stupid), visualisation_image[..., ::-1])
        # self.stupid += 1
        detection_summary_msg = ros_numpy.msgify(Image, visualisation_image, encoding="rgb8")
        self.detection_summary.publish(detection_summary_msg)


def detector_2d():
    rospy.init_node('bbc_countryfile_detector', anonymous=True)

    # get private namespace parameters
    p_camera_name = rospy.get_param('~camera_name', "bbc_camera")
    p_model_name = rospy.get_param('~model_name', "grid-rcnn")
    p_score_thresh = rospy.get_param('~score_thresh', 0.5)

    rospy.loginfo("Detector 2D: Initialising detector with camera: '{}', model '{}' and score thresh: '{}'".format(
        p_camera_name, p_model_name, p_score_thresh))

    detector = CounterBBC(camera_name=p_camera_name, model_name=p_model_name, score_thresh=p_score_thresh)
    rospy.spin()


if __name__ == '__main__':
    detector_2d()
