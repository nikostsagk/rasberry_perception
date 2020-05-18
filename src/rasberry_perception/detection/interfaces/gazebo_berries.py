#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com
from threading import Event

import ros_numpy
from rasberry_perception.msg import Detections, ServiceStatus, RegionOfInterest, SegmentOfInterest, Detection

from rasberry_perception.detection.interfaces.default import BaseDetectionServer
from rasberry_perception.detection.interfaces.registry import DETECTION_REGISTRY
from rasberry_perception.detection.utility import function_timer
from rasberry_perception.srv import GetDetectorResultsResponse, GetDetectorResultsRequest


class _unknown_class:
    def __init__(self):
        pass

    def __getitem__(self, item):
        return "class {}".format(item)


@DETECTION_REGISTRY.register_detection_backend("gazebo_berries")
class GazeboRenderedBerriesServer(BaseDetectionServer):
    def __init__(self):  # Put any required command line args here (see detectron2 for how it gets a config arg)
        try:
            # Do your imports here i.e import gazebo.get_berries
            pass
        except ImportError:
            raise

        self.currently_busy = Event()
        self.classes = _unknown_class()

        # Do initialisation code here

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
            return GetDetectorResultsResponse(status=ServiceStatus(BUSY=True))
        self.currently_busy.set()

        detections = Detections()

        try:
            image = ros_numpy.numpify(request.image)
            if request.image.encoding == "rgb8":
                image = image[..., ::-1]
            predictions = self.predictor(image)
            # Get the bounding boxes/detections for the image here
            self.currently_busy.clear()
            # Fill in the detections message
            return GetDetectorResultsResponse(status=ServiceStatus(OKAY=True), results=detections)
        except Exception as e:
            print("GazeboRenderedBerriesServer error: ", e)
            return GetDetectorResultsResponse(status=ServiceStatus(ERROR=True), results=detections)
