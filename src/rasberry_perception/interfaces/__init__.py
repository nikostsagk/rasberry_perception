#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

# Import all the interfaces here so that they're all registered
from .registry import DETECTION_REGISTRY
from .default import DefaultDetectionServer, default_service_name
from .detectron2 import Detectron2Server
from .gazebo_berries import GazeboRenderedBerriesServer
from .fruitcast import FruitCastServer

__all__ = ["FruitCastServer", "GazeboRenderedBerriesServer", "Detectron2Server", "DefaultDetectionServer"]
