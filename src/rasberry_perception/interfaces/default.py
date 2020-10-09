#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com
from threading import Event

import rospy

from rasberry_perception.utility import function_timer
from rasberry_perception.srv import GetDetectorResults, GetDetectorResultsResponse
from rasberry_perception.msg import ServiceStatus
from rasberry_perception.interfaces.registry import DETECTION_REGISTRY

default_service_name = "get_detections_service"


class BaseDetectionServer:
    """Base for all detection interfaces. Internal use only.

    The .___init___() method should be overridden and any interface initialisation done here such as loading models.
        Service name should not be changed unless multiple servers need to be run.
        Classes that inherit must call this baseclass at the end of their __init__ method or spin=False and call spin
        manually.
    The .get_detector_results() method must be overridden, this method should call the interface and translate the
        results to be in the rasberry_perception.msg.Detections message format.

    Args:
        service_name (str): Name of the created service. Should not be changed unless multiple servers need to be run.
        spin (bool): If True the service spin method called on construction. If false .spin() must be called later.
    """
    def __init__(self, service_name=default_service_name, spin=True):
        self._last_id = 0  # Last assigned ID
        self.service_name = service_name
        self.currently_busy = Event()
        rospy.loginfo("Creating service {}".format(service_name))
        self.service_server = rospy.Service(service_name, GetDetectorResults, self.get_detector_results)
        rospy.loginfo("Waiting for requests on {}".format(service_name))
        if spin:
            self.spin()

    def spin(self):
        self.service_server.spin()

    def get_detector_results(self, request):
        raise NotImplementedError("This is the base detector server intended for internal use only.")

    def _new_id(self):
        if self._last_id == 9223372036854775807:  # MAX_INT64
            self._last_id = 0
            return self._last_id
        self._last_id += 1
        return self._last_id - 1


@DETECTION_REGISTRY.register_detection_backend("default")
class DefaultDetectionServer(BaseDetectionServer):
    """Example of how to define a detection server interface.

    Every interface must register itself in the DETECTION_REGISTRY so ros is aware of it.

    Args:
        rate: The service will wait 1 / rate seconds before returning the default message
    """
    def __init__(self, rate=30):
        # Extra initialisation done here
        status_msg = ServiceStatus(OKAY=True, ERROR=False, BUSY=False)
        self.default_response = GetDetectorResultsResponse(status=status_msg)
        self._hz_limit = rospy.Rate(rate)
        # Base class must be called at the end due to self.service_server.spin()
        BaseDetectionServer.__init__(self)

    @function_timer.interval_logger(interval=10)
    def get_detector_results(self, request):
        self._hz_limit.sleep()
        return self.default_response
