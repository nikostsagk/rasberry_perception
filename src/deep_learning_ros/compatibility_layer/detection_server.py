#!/usr/bin/env python
from __future__ import absolute_import, division, print_function

import sys
from threading import Event

import rospy
from rasberry_perception.msg import ObjectDetection, BoundingBox
from rasberry_perception.srv import GetDetectorResults, GetDetectorResultsResponse

from deep_learning_ros.compatibility_layer.python3_fixes import KineticImportsFix
from deep_learning_ros.compatibility_layer.registry import DETECTION_REGISTRY
from rasberry_perception_pkg.utility import function_timer

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
        # Backbone specific imports
        try:
            import ros_numpy
            from os.path import abspath
            from collections import deque
            self.numpify = ros_numpy.numpify

            with KineticImportsFix():
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
        _DetectorResultsServer.__init__(self)

    @function_timer.interval_logger(interval=10)
    def get_detector_results(self, request):
        if self.currently_busy.is_set():
            return GetDetectorResultsResponse(status=DETECTOR_BUSY)
        self.currently_busy.set()

        response = GetDetectorResultsResponse(status=DETECTOR_FAIL)
        response.detections = ObjectDetection(header=request.image.header, class_labels=list(self.model.CLASSES))

        rgb_image = self.numpify(request.image)
        # Get results from detection backend and mark service as available (since GPU memory is the constraint here)
        result = self.inference_detector(self.model, rgb_image)
        self.currently_busy.clear()

        # Convert mmdetection results to annotation results
        # TODO: Add masks to response if they exist
        if isinstance(result, tuple):
            bounding_boxes = result[0]
            masks = result[1]
        else:
            bounding_boxes = result
            masks = None

        # Parse results into ObjectDetection method
        # Create bounding boxes list of [x1, y1, x2, y2, score, class_id]
        bounding_box_msgs = []
        for class_id, class_bounding_boxes in enumerate(bounding_boxes):
            for bbox in class_bounding_boxes:
                if bbox[-1] > request.score_thresh:
                    bounding_box_msgs.append(
                        BoundingBox(x1=bbox[0], y1=bbox[1], x2=bbox[2], y2=bbox[3], score=bbox[4], class_id=class_id))

        response.detections.bounding_boxes = bounding_box_msgs
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
    assigned_parameters = ["~backend"]

    # Fill in required backend arguments from the private ros parameter server
    kwargs = {}
    for arg_name in required_args:
        p_arg = "~" + arg_name
        if not rospy.has_param(p_arg):
            rospy.logerr("Parameter '{}' not found".format(arg_name))
            arg_list = " ".join(["_" + a + ":=<value>" for a in required_args])
            rospy.logerr("Backend '{}' requires rosrun parameters '{}'".format(backend, arg_list))
            sys.exit(1)
        assigned_parameters.append(p_arg)
        kwargs[arg_name] = rospy.get_param(p_arg)

    # Replace optional parameters if they exist
    for arg_name in optional_args:
        p_arg = "~" + arg_name
        if rospy.has_param(p_arg):
            assigned_parameters.append(p_arg)
            kwargs[arg_name] = rospy.get_param(p_arg)

    # Assign function to remove parameters on shutdown
    def delete_params_on_shutdown():
        for p in assigned_parameters:
            if rospy.has_param(p):
                rospy.delete_param(p)

    rospy.on_shutdown(delete_params_on_shutdown)

    # Get the backend
    server = DETECTION_REGISTRY[backend]

    try:
        # Start the server with the keyword arguments
        results_server = server(**kwargs)
    except (rospy.ROSInterruptException, KeyboardInterrupt) as e:
        rospy.logerr("Interrupt Received: Terminating Detection Server")


if __name__ == '__main__':
    __get_detector_results_server()
