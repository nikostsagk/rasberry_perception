#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

import sys
import rospy
from .interfaces import *


def __detection_server_runner():
    rospy.init_node("detection_server")
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
    __detection_server_runner()