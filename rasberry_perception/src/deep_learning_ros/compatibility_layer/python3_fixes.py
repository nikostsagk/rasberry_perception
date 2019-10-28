#!/usr/bin/env python
from __future__ import absolute_import, division, print_function
import sys


class KineticImportsFix:
    def __init__(self, kinetic_dist_packages="/opt/ros/kinetic/lib/python2.7/dist-packages"):
        self.kinetic_dist_packages = kinetic_dist_packages

    def __enter__(self):
        sys.path.remove(self.kinetic_dist_packages)

    def __exit__(self, exc_type, exc_val, exc_tb):
        sys.path.append(self.kinetic_dist_packages)
