from threading import Event
import ros_numpy
import time
import json
import numpy as np
from rasberry_perception.interfaces.default import BaseDetectionServer
from rasberry_perception.interfaces.registry import DETECTION_REGISTRY
from rasberry_perception.msg import Detections, ServiceStatus, RegionOfInterest, SegmentOfInterest, Detection
# from rasberry_perception.visualisation import GenericMask
from rasberry_perception.srv import GetDetectorResultsResponse, GetDetectorResultsRequest
from rasberry_perception.utility import function_timer

@DETECTION_REGISTRY.register_detection_backend("mmot")
class MMotServer(BaseDetectionServer):
    def __init__(self, config_path, image_height=480, image_width=640, image_hz=30):    
        try:
            import modularmot
            from modularmot.utils import ConfigDecoder
        except ImportError:
            raise
        with open(config_path) as cfg_file:
            config = json.load(cfg_file, cls=ConfigDecoder)        
        print("Load Engine")
        self.mot = modularmot.MOT([image_width, image_height],1.0/image_hz, config['mot'],
                          draw=False, verbose=False)
        #print("Passing Empty Frame to Init Engine")
        #self.mot.step(np.zeros((640,480,3), dtype=int))
        self.currently_busy = Event()
        
        # Base class must be called at the end due to self.service_server.spin()
        BaseDetectionServer.__init__(self)
        print("Done Init!")

    @staticmethod
    def citation_notice():
        return "Modular TensorRT Inference and Feature Extractor\n" \
               "Maintained by Robert Belshaw (rbelshaw@sagarobotics.com)"

    @function_timer.interval_logger(interval=10)
    def get_detector_results(self, request):
        """
        Args:
            request (GetDetectorResultsRequest):
        Returns:
            GetDetectorResultsResponse
        """
        try:
            import numpy as np
        except ImportError:
            raise
        
        if self.currently_busy.is_set():
            return GetDetectorResultsResponse(status=ServiceStatus(BUSY=True))
        self.currently_busy.set()
        detections = Detections()
        try:
            image = ros_numpy.numpify(request.image)
            if request.image.encoding == "rgb8":
                image = image[..., ::-1]
            self.mot.step(image)
            for track in self.mot.visible_tracks:
                confidence = 0.99
                if track.label == 0:
                    class_name = "Ripe Strawberry"
                else: 
                    class_name = "Unripe Strawberry"
                track_id = track.trk_id
                roi = RegionOfInterest(x1=track.tlbr[0], y1=track.tlbr[1], x2=track.tlbr[2], y2=track.tlbr[3])
                seg_roi = SegmentOfInterest(x=[], y=[])
                detections.objects.append(Detection(roi=roi, seg_roi=seg_roi, id=self._new_id(), track_id=track_id,
                                                    confidence=confidence, class_name=class_name))
        except Exception as e:
            self.currently_busy.clear()
            print("FruitCastServer error: ", e)
            return GetDetectorResultsResponse(status=ServiceStatus(ERROR=True), results=detections)
        self.currently_busy.clear()
        return GetDetectorResultsResponse(status=ServiceStatus(OKAY=True), results=detections)