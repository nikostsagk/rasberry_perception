#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com
from threading import Event

import rospy
from rasberry_perception.srv import GetDetectorResults, GetDetectorResultsResponse
from rasberry_perception.msg import DetectionStatus
from rasberry_perception.detection.interfaces.registry import DETECTION_REGISTRY


class BaseDetectionServer:

    def __init__(self, service_name="get_detections_service"):
        self.service_name = service_name
        self.currently_busy = Event()
        rospy.loginfo("Creating service {}".format(service_name))
        self.service_server = rospy.Service(service_name, GetDetectorResults, self.get_detector_results)
        rospy.loginfo("Waiting for requests on {}".format(service_name))
        self.service_server.spin()

    def get_detector_results(self, request):
        raise NotImplementedError("This is the base detector server intended for internal use only.")


@DETECTION_REGISTRY.register_detection_backend("default")
class DefaultDetectionServer(BaseDetectionServer):
    def __init__(self):
        BaseDetectionServer.__init__(self)

    def get_detector_results(self, request):
        status_msg = DetectionStatus(OKAY=True, ERROR=False, BUSY=False)
        return GetDetectorResultsResponse(status=status_msg)
