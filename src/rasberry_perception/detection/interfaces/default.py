#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com
from threading import Event

import rospy

from rasberry_perception.detection.utility import function_timer
from rasberry_perception.srv import GetDetectorResults, GetDetectorResultsResponse
from rasberry_perception.msg import DetectionStatus
from rasberry_perception.detection.interfaces.registry import DETECTION_REGISTRY

default_service_name = "get_detections_service"


class BaseDetectionServer:
    """Base for all detection interfaces. Internal use only.

    The .___init___() method should be overridden and any interface initialisation done here such as loading models.
        Service name should not be changed unless multiple servers need to be run.
        Classes that inherit must call this baseclass at the end of their __init__ method.
    The .get_detector_results() method must be overridden, this method should call the interface and translate the
        results to be in the rasberry_perception.msg.Detections message format.
    """
    def __init__(self, service_name=default_service_name):
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
    """Example of how to define a detection server interface.

    Every interface must register itself in the DETECTION_REGISTRY so ros is aware of it.
    """
    def __init__(self):
        # Extra initialisation done here
        status_msg = DetectionStatus(OKAY=True, ERROR=False, BUSY=False)
        self.default_response = GetDetectorResultsResponse(status=status_msg)
        # Base class must be called at the end due to self.service_se
        BaseDetectionServer.__init__(self)

    @function_timer.interval_logger(interval=10)
    def get_detector_results(self, request):
        return self.default_response
