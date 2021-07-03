#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

# Import all the interfaces here so that they're all registered
from .registry import DETECTION_REGISTRY
from .default import DefaultDetectionServer, default_service_name
from .detectron2 import Detectron2Server
from .gazebo_berries import GazeboRenderedBerriesServer
from .fruitcast import FruitCastServer
from .mmot import MMotServer
from .yolov4deepsort import YoloV4DeepsortServer
from .tensorrtdeepsort import TensorrtDeepsortServer

__all__ = ["YoloV4DeepsortServer","MMotServer","TensorrtDeepsortServer", "FruitCastServer", "GazeboRenderedBerriesServer", "Detectron2Server", "DefaultDetectionServer"]
