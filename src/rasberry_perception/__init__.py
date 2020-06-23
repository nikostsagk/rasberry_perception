# -*- coding: utf-8 -*-
#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

"""
rasberry_perception python ROS package for bridging the gap between perception backends and ROS interfaces
See LICENSE. Originally written and designed by raymond
"""

__author__ = 'Raymond Kirk (Tunstill)'
__maintainer__ = __author__
__email__ = "ray.tunstill@gmail.com"
__copyright__ = 'Copyright 2020, rasberry_perception'
__license__ = 'See LICENCE in project root'
__version__ = '0.0.0'

from .interfaces import (DETECTION_REGISTRY,
                         default_service_name)
from .service import (Server,
                      Client)
from . import compat
