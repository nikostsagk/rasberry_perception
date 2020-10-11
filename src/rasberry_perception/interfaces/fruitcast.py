#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com
from threading import Event

import ros_numpy

from rasberry_perception.interfaces.default import BaseDetectionServer
from rasberry_perception.interfaces.registry import DETECTION_REGISTRY
from rasberry_perception.msg import Detections, ServiceStatus, RegionOfInterest, SegmentOfInterest, Detection
# from rasberry_perception.visualisation import GenericMask
from rasberry_perception.srv import GetDetectorResultsResponse, GetDetectorResultsRequest
from rasberry_perception.utility import function_timer


@DETECTION_REGISTRY.register_detection_backend("fruitcast")
class FruitCastServer(BaseDetectionServer):
    def __init__(self, weights, image_size=640, nms_conf_thresh=0.4, nms_iou_thresh=0.5, device=""):
        try:
            from yolov5.utils.general import set_logging
            from yolov5.utils.torch_utils import select_device
            from yolov5.models.experimental import attempt_load
            from yolov5.utils.general import check_img_size
            from yolov5.utils.general import non_max_suppression
            from yolov5.utils.general import scale_coords
            from yolov5.utils.datasets import letterbox
        except ImportError:
            raise
        self.device = select_device(device)
        self.conf_thresh = nms_conf_thresh
        self.iou_thresh = nms_iou_thresh
        self.half = self.device.type != 'cpu'
        self.model = attempt_load(weights, map_location=self.device)
        self.image_size = check_img_size(image_size, s=self.model.stride.max())
        if self.half:
            self.model.half()  # to FP16
        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names

        self.currently_busy = Event()

        # Base class must be called at the end due to self.service_server.spin()
        BaseDetectionServer.__init__(self)

    @staticmethod
    def citation_notice():
        return "Please cite this work as outlined in https://github.com/RaymondKirk/{fruit_detection,fruitcast}\n" \
               "Maintained by Raymond Kirk (ray.tunstill@gmail.com)"

    @function_timer.interval_logger(interval=10)
    def get_detector_results(self, request):
        """

        Args:
            request (GetDetectorResultsRequest):

        Returns:
            GetDetectorResultsResponse
        """
        try:
            import torch
            from yolov5.utils.general import non_max_suppression
            from yolov5.utils.general import scale_coords
            from yolov5.utils.datasets import letterbox
            import numpy as np
        except ImportError:
            raise

        if self.currently_busy.is_set():
            return GetDetectorResultsResponse(status=ServiceStatus(BUSY=True))
        self.currently_busy.set()

        detections = Detections()

        try:
            image = ros_numpy.numpify(request.image)
            if request.image.encoding == "rgb8":
                image = image[..., ::-1]

            original_shape = image.shape
            img = letterbox(image, new_shape=self.image_size)[0]
            img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB
            img = np.ascontiguousarray(img)

            img = torch.from_numpy(img).to(self.device)
            img = img.half() if self.half else img.float()  # uint8 to fp16/32
            img /= 255.0  # 0 - 255 to 0.0 - 1.0
            if img.ndimension() == 3:
                img = img.unsqueeze(0)
            with torch.no_grad():
                pred = self.model(img, augment=False)[0]
            pred = non_max_suppression(pred, self.conf_thresh, self.iou_thresh, agnostic=False)

            for i, det in enumerate(pred):
                if det is not None and len(det):
                    det[:, :4] = scale_coords(img.shape[2:], det[:, :4], original_shape).round()

                    for x1, y1, x2, y2, conf, cls in reversed(det):
                        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                        confidence = float(conf)
                        class_name = self.names[int(cls)]
                        roi = RegionOfInterest(x1=x1, y1=y1, x2=x2, y2=y2)
                        seg_roi = SegmentOfInterest(x=[], y=[])
                        detections.objects.append(Detection(roi=roi, seg_roi=seg_roi, id=self._new_id(), track_id=-1,
                                                            confidence=confidence, class_name=class_name))
                self.currently_busy.clear()
        except Exception as e:
            print("FruitCastServer error: ", e)
            return GetDetectorResultsResponse(status=ServiceStatus(ERROR=True), results=detections)

        return GetDetectorResultsResponse(status=ServiceStatus(OKAY=True), results=detections)
