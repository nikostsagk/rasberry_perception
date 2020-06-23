#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

# File should provide functionality to 'lubricate' python support in 2/3 environments between ROS/DL environments"""

from __future__ import absolute_import, division, print_function
import sys

try:
    from Queue import Queue
except ImportError:
    from queue import Queue

try:
    input = raw_input
except NameError:
    pass


class RosImportsFix:
    """Context manager to ignore python libraries installed in ROS distros.

    Backwards compatible with kinetic and melodic

    Examples:
        >>> with RosImportsFix():
        >>>     import library_you_really_need_but_sucks_in_ROS  # i.e opencv2, torch etc.
    """
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
