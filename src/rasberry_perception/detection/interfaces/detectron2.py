#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com
from threading import Event

import numpy as np
import ros_numpy
from rasberry_perception.msg import Detections, DetectionStatus, RegionOfInterest, SegmentOfInterest, Detection, \
    DetectionInfo

from rasberry_perception.detection.interfaces.default import BaseDetectionServer
from rasberry_perception.detection.interfaces.registry import DETECTION_REGISTRY
from rasberry_perception.detection.utility import function_timer
from rasberry_perception.detection.visualisation import GenericMask
from rasberry_perception.srv import GetDetectorResultsResponse, GetDetectorResultsRequest


class _unknown_class:
    def __init__(self):
        pass

    def __getitem__(self, item):
        return "unknown"


@DETECTION_REGISTRY.register_detection_backend("detectron2")
class Detectron2Server(BaseDetectionServer):
    _supported_revision = "2a571ea55bd3111118063c666735bc81d36ff4de"
    _supported_version = "0.1.1"

    def __init__(self, config_file, model_file=None):
        try:
            from detectron2 import __version__ as version
            if version != self._supported_version:
                raise RuntimeError("Supported version is '{}', you have '{}'.".format(self._supported_version, version))
            from detectron2.config import get_cfg
            from detectron2.data import MetadataCatalog
            from detectron2.engine.defaults import DefaultPredictor
            # from fruit_detection.config import add_fruit_detection_config
        except ImportError:
            raise

        self.currently_busy = Event()
        self.cfg = get_cfg()
        # add_fruit_detection_config(self.cfg)
        self.cfg.merge_from_file(config_file)
        if model_file is not None:
            self.cfg.MODEL.WEIGHTS = model_file
        self.cfg.freeze()
        metadata = MetadataCatalog.get(self.cfg.DATASETS.TEST[0] if len(self.cfg.DATASETS.TEST) else "__unused")
        self.classes = metadata.get("thing_classes") or _unknown_class()
        self.predictor = DefaultPredictor(self.cfg)

        # Base class must be called at the end due to self.service_server.spin()
        BaseDetectionServer.__init__(self)

    @function_timer.interval_logger(interval=10)
    def get_detector_results(self, request):
        """

        Args:
            request (GetDetectorResultsRequest):

        Returns:
            GetDetectorResultsResponse
        """
        if self.currently_busy.is_set():
            return GetDetectorResultsResponse(status=DetectionStatus(BUSY=True))
        self.currently_busy.set()

        detections = Detections(header=request.image.header)

        try:
            image = ros_numpy.numpify(request.image)
            predictions = self.predictor(image)
            self.currently_busy.clear()

            if "instances" in predictions:
                instances = predictions["instances"].to("cpu")
                boxes = np.asarray(instances.pred_boxes.tensor) if instances.has("pred_boxes") else None

                if len(boxes) == 0:
                    return GetDetectorResultsResponse(status=DetectionStatus(), detections=detections)

                scores = instances.scores if instances.has("scores") else None
                classes = instances.pred_classes if instances.has("pred_classes") else None
                labels = ["{} {}".format(self.classes[cls_id], scores[idx]) for idx, cls_id in enumerate(classes)]
                masks = np.asarray(instances.pred_masks) if instances.has("pred_masks") else [None] * len(boxes)

                # if instances.has("pred_classes"):
                #     masks = [GenericMask(x, *x.shape) for x in np.asarray(instances.pred_masks)]
                # else:
                #     masks = [None] * len(boxes)

                for score, cls, bbox, mask in zip(scores, classes, boxes, masks):
                    x1, y1, x2, y2 = bbox
                    yv, xv = [], []
                    if mask is not None:
                        v = np.where(mask != 0)
                        if v:
                            yv, xv = v
                    roi = RegionOfInterest(x1=x1, y1=y1, x2=x2, y2=y2)
                    seg_roi = SegmentOfInterest(x=xv, y=yv)
                    info = DetectionInfo(score=score, class_id=cls, class_name=self.classes[cls])
                    detections.detections.append(Detection(roi=roi, seg_roi=seg_roi, info=info))
        except Exception as e:
            print("Detectron2Server error: ", e)
            return GetDetectorResultsResponse(status=DetectionStatus(ERROR=True), detections=detections)

        return GetDetectorResultsResponse(status=DetectionStatus(OKAY=True), detections=detections)
