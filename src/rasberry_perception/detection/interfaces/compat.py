#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

from __future__ import absolute_import, division, print_function
import sys


class RosImportsFix:
    def __init__(self, ros_distros=None):
        if ros_distros is None:
            ros_distros = ["/opt/ros/{}/lib/python2.7/dist-packages".format(distro) for distro in ["kinetic",
                                                                                                   "melodic"]]
        self.dist_package_paths = ros_distros
        self._removed_distros = []

    def __enter__(self):
        self._removed_distros = []
        for distro_path in self.dist_package_paths:
            if distro_path in sys.path:
                self._removed_distros.append(distro_path)
                sys.path.remove(distro_path)

    def __exit__(self, exc_type, exc_val, exc_tb):
        for distro_path in self._removed_distros:
            sys.path.append(distro_path)
        self._removed_distros = []
