#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

import sys
import rospy

from rasberry_perception.srv import GetDetectorResults
from rasberry_perception.detection import DETECTION_REGISTRY, default_service_name


class Server:
    """Launches detection server with self configuring backend based on the rosparam server

    Parses private and node parameters to extract backend information and custom backend args. If the interface backend
        class requires two parameters x and y then this class will extract this information if it exists from ROS and
        launch the requested server

    Args:
        backend (str): Name of the backend to initialise. If None it's extracted from the ros param server.
        backend_kwargs (dict): Appropriate arguments required by the backend. If none they're defined by the ros
            param server.
        service_name (str): Underlying service name to start based on (shouldn't change from
            rasberry_perception.detection.default_service_name) unless overridden in custom interface backend

    Attributes:
        server_args (dict): Key, Value pairs of appropriate ros parameters for the chosen backend. Default would be
            {'backend': 'default'}
        backend_name (str): Name of the chosen backend
        server (BaseDetectionServer): Class interface of the server

    Examples:
        >>> server = Server()
        >>> server.run()

    Raises:
        rospy.ROSException: If detection service proxy can't be configured correctly
        rospy.ServiceException: If detection server can't be called
    """
    def __init__(self, backend=None, backend_kwargs=None, service_name=default_service_name):
        self._node_name = service_name + "_server"
        rospy.init_node(self._node_name)

        if backend is None:
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

        if backend_kwargs is None:
            # Fill in required backend arguments from the private ros parameter server
            backend_kwargs = {}
            for arg_name in required_args:
                p_arg = "~" + arg_name
                if not rospy.has_param(p_arg):
                    rospy.logerr("Parameter '{}' not found".format(arg_name))
                    arg_list = " ".join(["_" + a + ":=<value>" for a in required_args])
                    rospy.logerr("Backend '{}' requires rosrun parameters '{}'".format(backend, arg_list))
                    sys.exit(1)
                assigned_parameters.append(p_arg)
                backend_kwargs[arg_name] = rospy.get_param(p_arg)

            # Replace optional parameters if they exist
            for arg_name in optional_args:
                p_arg = "~" + arg_name
                if rospy.has_param(p_arg):
                    assigned_parameters.append(p_arg)
                    backend_kwargs[arg_name] = rospy.get_param(p_arg)

        self.server_args = backend_kwargs
        self.backend_name = backend
        self._assigned_parameters = assigned_parameters

        # Register callback to delete parameters from ROS param server on shutdown
        rospy.on_shutdown(self.on_shutdown)

        # Get the backend
        self.server = DETECTION_REGISTRY[self.backend_name]

    def on_shutdown(self):
        # Remove parameters on shut down (params should not persist between servers)
        for p in self._assigned_parameters:
            if rospy.has_param(p):
                rospy.delete_param(p)

    def run(self):
        try:
            # Start the server with the keyword arguments
            self.server(**self.server_args)
        except (rospy.ROSInterruptException, KeyboardInterrupt) as e:
            rospy.logerr(e)
            rospy.logerr("Interrupt Received: Exiting '{}' node".format(self._node_name))


class Client:
    """Client to interact with the detection server via the GetDetectorResults ROS service API

    Args:
        timeout (int): How long to wait between polling for the service to become available
        service_name (str): Underlying service name to connect to (shouldn't change from
            rasberry_perception.detection.default_service_name) unless overridden in custom interface backend

    Examples:
        >>> client = Client()
        >>> result = client(image=sensor_msgs.msgs.Image(), score_thresh=0.5)

    Raises:
        rospy.ROSException: If detection service proxy can't be configured correctly
        rospy.ServiceException: If detection server can't be called
    """
    def __init__(self, timeout=10, service_name=default_service_name):
        self.detection_server = None
        self._service_name = service_name
        self.__timeout = timeout
        self._connect()

    def _connect(self):
        while not rospy.is_shutdown():
            try:
                rospy.loginfo("Waiting for '{}' service".format(self._service_name))
                rospy.wait_for_service(self._service_name, timeout=self.__timeout)
                self.detection_server = rospy.ServiceProxy(self._service_name, GetDetectorResults)
                return
            except rospy.ROSException as e:
                rospy.logerr(e)

    def _get_result(self, *args, **kwargs):
        try:
            return self.detection_server(*args, **kwargs), True
        except rospy.ServiceException as e:
            rospy.logerr(e)
            self._connect()
            return None, False

    def __call__(self, *args, **kwargs):
        result, success = self._get_result(*args, **kwargs)
        while not success:
            result, success = self._get_result(*args, **kwargs)
        return result
