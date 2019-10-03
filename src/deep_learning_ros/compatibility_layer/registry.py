#!/usr/bin/env python
from __future__ import absolute_import, division, print_function

import inspect

import rospy


class __DetectionRegistry:
    def __init__(self):
        self._modules = {}

    def register_detection_backend(self, backend_name):
        def class_registration_decorator(module_class):
            # Ensure all registry items are of the same type
            if len(self._modules):
                required_type = type(self._modules[list(self._modules.keys())[0]])
                if not isinstance(module_class, required_type):
                    raise ValueError(
                        "Backend module '{}' must inherit from _DetectorResultsServer".format(module_class.__name__))
            if backend_name in self._modules:
                rospy.logerr("Backend '{}' already in the detection registry as '{}'".format(
                    backend_name, self._modules[backend_name].name))
            else:
                rospy.loginfo("Registering backend '{}' as '{}'".format(module_class.__name__, backend_name))
                self._modules[backend_name] = module_class

            return self._modules[backend_name]

        return class_registration_decorator

    def __getitem__(self, item):
        return self._modules[item]

    def __contains__(self, item):
        return item in self._modules

    def available_backends(self):
        return tuple(self._modules.keys())

    def get_arguments(self, item):
        cls = self._modules[item]
        args, _, _, defaults = inspect.getargspec(cls)
        if defaults is None:
            defaults = []
        if args is None:
            args = []
        if "self" in args:
            args.remove("self")
        n_defaults = 0 if defaults is None else len(defaults)
        required_args = list(args[:-n_defaults])
        optional_args = dict(zip(args[-n_defaults:], defaults))
        return required_args, optional_args


DETECTION_REGISTRY = __DetectionRegistry()
