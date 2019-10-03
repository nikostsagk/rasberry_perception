#!/usr/bin/env python
from __future__ import absolute_import, division, print_function

import os
import sys
from threading import Event

import ros_numpy
import rospy
from rasberry_perception.msg import ObjectDetection
from rasberry_perception.srv import GetDetectorResults, GetDetectorResultsResponse

from deep_learning_ros.compatibility_layer.python3_fixes import KineticImportsFix
from deep_learning_ros.compatibility_layer.registry import DETECTION_REGISTRY

DETECTOR_OK = "OKAY"
DETECTOR_FAIL = "FAIL"
DETECTOR_BUSY = "BUSY"
_detector_service_name = "get_detection_results"


class DetectorResultsClient:
    def __init__(self, timeout=10):
        rospy.loginfo("Waiting for '{}' service".format(_detector_service_name))
        rospy.wait_for_service(_detector_service_name, timeout=timeout)
        self.detection_server = rospy.ServiceProxy(_detector_service_name, GetDetectorResults)

    def __call__(self, *args, **kwargs):
        return self.detection_server(*args, **kwargs)


class _DetectorResultsServer:
    def __init__(self):
        self.currently_busy = Event()
        rospy.loginfo("Creating service {}".format(_detector_service_name))
        self.service_server = rospy.Service(_detector_service_name, GetDetectorResults, self.get_detector_results)
        rospy.loginfo("Waiting for requests on {}".format(_detector_service_name))
        self.service_server.spin()

    def get_detector_results(self, request):
        raise NotImplementedError()


@DETECTION_REGISTRY.register_detection_backend("default")
class _DefaultDetectorResultsServer(_DetectorResultsServer):
    def __init__(self):
        _DetectorResultsServer.__init__(self)

    def get_detector_results(self, request):
        return GetDetectorResultsResponse(status=DETECTOR_OK)


@DETECTION_REGISTRY.register_detection_backend("mmdetection")
class _MMDetectionResultsServer(_DetectorResultsServer):
    def __init__(self, config_path, model_path, device=None):
        # Import in main class definition to allow importing this file in Python2 files
        self.inference_detector = None
        with KineticImportsFix():
            try:
                import torch
                from mmdet.apis import inference_detector, init_detector
                self.inference_detector = inference_detector  # Export function to class scope
            except ImportError as e:
                rospy.logerr(e)
                rospy.logerr("Please source your backend detection environment before running the detection service.")
                sys.exit(1)

        config_path = os.path.abspath(config_path)
        model_path = os.path.abspath(model_path)

        # Initialise detection backend
        if device is None:
            rospy.loginfo("No device specified, defaulting to first available CUDA device")
            device = torch.device('cuda', 0)

        rospy.loginfo("Initialising model with config '{}' and model '{}'".format(config_path, model_path))
        self.model = init_detector(config_path, model_path, device=device)

        # Initialise ros service interface (done last so callback isn't called before setup)
        _DetectorResultsServer.__init__(self)

    def get_detector_results(self, request):
        if self.currently_busy.is_set():
            return GetDetectorResultsResponse(status=DETECTOR_BUSY)
        self.currently_busy.set()
        response = GetDetectorResultsResponse(status=DETECTOR_FAIL)
        response.detections = ObjectDetection()

        rgb_image = ros_numpy.numpify(request.image)
        # Get results from detection backend and mark service as available (since GPU memory is the constraint here)
        result = self.inference_detector(self.model, rgb_image)
        self.currently_busy.clear()

        # Parse results into ObjectDetection method
        response.status = DETECTOR_OK
        return response


def __get_detector_results_server():
    rospy.init_node(_detector_service_name + "_server")
    backend = rospy.get_param('~backend', "default")
    if backend not in DETECTION_REGISTRY:
        rospy.logerr(
            "Backend '{}' not in registry see README.md file and add it as a backend! Available backends: {}".format(
                backend, DETECTION_REGISTRY.available_backends()))
        sys.exit(1)

    rospy.loginfo("Configuring detection service for backend '{}'".format(backend))

    # Parse passed parameters (fail if required missing, override if optional present)
    required_args, optional_args = DETECTION_REGISTRY.get_arguments(backend)

    # Fill in required backend arguments from the private ros parameter server
    kwargs = {}
    for arg_name in required_args:
        if not rospy.has_param("~" + arg_name):
            rospy.logerr("Parameter '{}' not found".format(arg_name))
            arg_list = " ".join(["_" + a + ":=<value>" for a in required_args])
            rospy.logerr("Backend '{}' requires rosrun parameters '{}'".format(backend, arg_list))
            sys.exit(1)
        kwargs[arg_name] = rospy.get_param("~" + arg_name)

    # Replace optional parameters if they exist
    for arg_name in optional_args:
        if rospy.has_param("~" + arg_name):
            kwargs[arg_name] = rospy.get_param("~" + arg_name)

    # Get the backend
    server = DETECTION_REGISTRY[backend]

    try:
        # Start the server with the keyword arguments
        results_server = server(**kwargs)
    except (rospy.ROSInterruptException, KeyboardInterrupt) as e:
        rospy.logerr("Interrupt Received: Terminating Detection Server")


if __name__ == '__main__':
    __get_detector_results_server()
