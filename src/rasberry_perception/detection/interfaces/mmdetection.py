#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com
import sys

import rospy

from rasberry_perception.detection.interfaces.default import BaseDetectionServer
from rasberry_perception.detection.interfaces.registry import DETECTION_REGISTRY
from rasberry_perception.srv import GetDetectorResults, GetDetectorResultsResponse
from rasberry_perception.msg import DetectionStatus, Detections, SegmentOfInterest, RegionOfInterest
from rasberry_perception.detection.compat import RosImportsFix
from rasberry_perception.detection.utility import function_timer


@DETECTION_REGISTRY.register_detection_backend("mmdetection")
class MMDetectionServer(BaseDetectionServer):
    def __init__(self, config_path, model_path, device=None):
        try:
            # Backbone specific imports (local import to not break python2.7 environment)
            import ros_numpy
            from os.path import abspath
            from collections import deque
            import pycocotools.mask as maskUtils
            import numpy as np

            self.mask_decode = lambda x: np.where(maskUtils.decode(x).astype(np.bool))
            self.numpify = ros_numpy.numpify

            # Ensures the ROS python lib paths aren't searched for packages (cv2 and torch cv2 conflict)
            with RosImportsFix():
                import torch
                from mmdet.apis import inference_detector, init_detector
                self.inference_detector = inference_detector  # Export function to class scope
        except ImportError as e:
            rospy.logerr(e)
            rospy.logerr("Please source your backend detection environment before running the detection service.")
            sys.exit(1)

        config_path = abspath(config_path)
        model_path = abspath(model_path)

        # Initialise detection backend
        if device is None:
            rospy.loginfo("No device specified, defaulting to first available CUDA device")
            device = torch.device('cuda', 0)

        rospy.loginfo("Initialising model with config '{}' and model '{}'".format(config_path, model_path))
        self.model = init_detector(config_path, model_path, device=device)

        # Initialise ros service interface (done last so callback isn't called before setup)
        BaseDetectionServer.__init__(self)

    @function_timer.interval_logger(interval=10)
    def get_detector_results(self, request):
        if self.currently_busy.is_set():
            return GetDetectorResultsResponse(status=DetectionStatus(BUSY=True))
        self.currently_busy.set()

        response = GetDetectorResultsResponse(status=DetectionStatus(ERROR=True))
        class_labels = list(self.model.CLASSES)
        response.detections = Detections(header=request.image.header)

        rgb_image = self.numpify(request.image)
        # Get results from detection backend and mark service as available (since GPU memory is the constraint here)
        result = self.inference_detector(self.model, rgb_image)
        self.currently_busy.clear()

        # Convert mmdetection results to annotation results
        if isinstance(result, tuple):
            # Response should be bboxes = [[x1, y1, x2, y2, score, class_id] ...]
            bounding_boxes = result[0]
            masks = result[1]
            if len(bounding_boxes) != len(masks) or len(bounding_boxes[0]) != len(masks[0]):
                rospy.logerr("Bounding boxes and masks are of different lengths {} and {}".format(len(bounding_boxes),
                                                                                                  len(masks)))
                return response
        else:
            bounding_boxes = result
            masks = None

        # Parse results into ObjectDetection method
        # Create bounding boxes list of [x1, y1, x2, y2, score, class_id]
        n_classes = len(bounding_boxes)
        for class_id in range(n_classes):
            n_detections = len(bounding_boxes[class_id])
            for detection_id in range(n_detections):
                bbox = bounding_boxes[class_id][detection_id]
                mask = None if masks is None else masks[class_id][detection_id]

                if bbox[-1] > request.score_thresh:
                    det_info = DetectionInfo(score=bbox[4], class_id=class_id, class_name=class_labels[class_id],
                                             detection_id="")
                    det = Detection(info=det_info)
                    det.roi = RegionOfInterest(x1=bbox[0], y1=bbox[1], x2=bbox[2], y2=bbox[3])
                    if mask is not None:
                        y_p, x_p = self.mask_decode(mask)
                        det.seg_roi = SegmentOfInterest(x=x_p, y=y_p, score=bbox[4], class_id=class_id)
                    response.detections.detections.append(det)
        response.status = DetectionStatus(OKAY=True)
        return response
