#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com
from threading import Event

import cv2
import ros_numpy

from rasberry_perception.interfaces.default import BaseDetectionServer
from rasberry_perception.interfaces.registry import DETECTION_REGISTRY
from rasberry_perception.msg import Detections, ServiceStatus, RegionOfInterest, SegmentOfInterest, Detection
# from rasberry_perception.visualisation import GenericMask
from rasberry_perception.srv import GetDetectorResultsResponse, GetDetectorResultsRequest
from rasberry_perception.utility import function_timer


@DETECTION_REGISTRY.register_detection_backend("unet")
class UNetServer(BaseDetectionServer):
    def __init__(self, model_path, config_path, image_size=640, nms_conf_thresh=0.4, nms_iou_thresh=0.5, device=""):
        try:
            import torch
            print(torch.cuda.is_available())
            from berry_segmentation.config import UNetConfig
            from berry_segmentation.unet import NestedUNet
            from berry_segmentation.unet import UNet
            import yaml
        except ImportError:
            raise
        print('model_path: '+model_path)
        print('config_path: '+config_path)
        with open(config_path, 'r') as stream:
            self.config = yaml.load(stream)
        self.net = eval(self.config.model)(self.config)
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.net.to(device=self.device)
        self.net.load_state_dict(torch.load(model_path, map_location=self.device))


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
            import numpy as np
            from berry_segmentation.inference_color import inference_one, visualise_mask
            import PIL
            import cv2
            from skimage import measure


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

            img = PIL.Image.fromarray(image.astype('uint8'), 'RGB')

            mask = inference_one(net=self.net,
                                 image=img,
                                 device=self.device,
                                 config=self.config)
            for m, c in zip(mask,self.config.class_names):
                m = m.astype('int')
                # seg_roi = cv2.findContours(m, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
                contours = measure.find_contours(m, 0.5)
                # for contour in contours:
                #     contour = np.flip(contour, axis=1)
                #     segmentation = contour.ravel().tolist()
                if contours == []:
                    seg_roi = SegmentOfInterest(x=[],y=[])
                    roi=RegionOfInterest(x1=0,x2=0,y1=0,y2=0)
                else:
                    x_vals = [int(coord[0]) for coord in contours[0]]
                    y_vals = [int(coord[1]) for coord in contours[0]]
                    seg_roi = SegmentOfInterest(x=x_vals,y=y_vals)
                    roi=RegionOfInterest(x1=min(x_vals),y1=min(y_vals),x2=max(x_vals),y2=max(y_vals))
                detections.objects.append(Detection(roi=roi,seg_roi=seg_roi, class_name=c, confidence=1,id=self._new_id(),track_id=1))


            self.currently_busy.clear()
        except Exception as e:
            print("FruitCastServer error: ", e)
            return GetDetectorResultsResponse(status=ServiceStatus(ERROR=True), results=detections)

        return GetDetectorResultsResponse(status=ServiceStatus(OKAY=True), results=detections)
