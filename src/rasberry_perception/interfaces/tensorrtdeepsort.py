from codecs import encode
from threading import Event
import ros_numpy
import json
from rasberry_perception.interfaces.default import BaseDetectionServer
from rasberry_perception.interfaces.registry import DETECTION_REGISTRY
from rasberry_perception.msg import Detections, ServiceStatus, RegionOfInterest, SegmentOfInterest, Detection
from rasberry_perception.srv import GetDetectorResultsResponse, GetDetectorResultsRequest
from rasberry_perception.utility import function_timer

@DETECTION_REGISTRY.register_detection_backend("tensorrtdeepsort")
class TensorrtDeepsortServer(BaseDetectionServer):
    def __init__(self, config_path, service_name, image_height=480, image_width=640, image_hz=30, deepsort_modelPath="/mars_sb_14.pb",max_cosine_distance = 0.6, nn_budget = 50, nms_max_overlap = 1.0):    
        try:
            import modularmot
            from modularmot.utils import ConfigDecoder
            from deep_sort.tracker import Tracker
            from deep_sort import nn_matching
            from tools import generate_detections as gdet
            from deep_sort import preprocessing
            from deep_sort.detection import Detection as deep_detection 
            import os
        except ImportError:
            raise
        with open(config_path) as cfg_file:
            config = json.load(cfg_file, cls=ConfigDecoder)        
        project_dir = os.path.dirname(__file__)
        deepsort_modelPath = os.path.join(project_dir, deepsort_modelPath)
        self.nms_max_overlap = nms_max_overlap        
        
        #Image Info
        self.image_height = image_height
        self.image_width = image_width
        
        print("Load Engine")
        self.mot = modularmot.MOT([int(image_width), int(image_height)],1.0/int(image_hz), config['mot'], detections_only=True, verbose=False)
        # deep_sort
        self.preprocessing = preprocessing
        self.deep_detection = deep_detection
        self.encoder = gdet.create_box_encoder(deepsort_modelPath, batch_size=1)
        metric = nn_matching.NearestNeighborDistanceMetric("cosine", max_cosine_distance, nn_budget)
        self.tracker = Tracker(metric)   
        self.currently_busy = Event()
        
        # Base class must be called at the end due to self.service_server.spin()
        BaseDetectionServer.__init__(self, service_name=service_name)

    @staticmethod
    def citation_notice():
        return "TensorRT Inference and Feature Extrator\n" \
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
        detections_msg = Detections()        
        try:
            image = ros_numpy.numpify(request.image)
            if request.image.encoding == "rgb8":
                image = image[..., ::-1]
            #Image Info
            self.image_height, self.image_width = image.shape[0], image.shape[1] 

            self.mot.step(image)
            # detections
            ndetections = len(self.mot.detections)
            boxs = np.empty(shape=(ndetections, 4))
            confidences = np.empty(shape=(ndetections,))
            class_name = np.ones(ndetections)

            # tracking
            deep_detections = []

            for n, detection in enumerate(self.mot.detections):
                x1, y1, x2, y2 = detection[0]
                h = y2 - y1
                w = x2 - x1
                boxs[n,:] = int(x1), int(y1), int(w), int(h)
                confidences[n] = detection[2]
                class_name[n] = detection[1]

                if detection[1] != 0:  # only track ripe berry whose class is "ripe"        
                    detections_msg.objects.append(Detection(roi=RegionOfInterest(x1=x1, y1=y1, x2=x2, y2=y2), seg_roi=SegmentOfInterest(x=[], y=[]), id=self._new_id(), track_id=-1, confidence=detection[2], class_name="unripe"))                        
                    continue
                
                else:
                    features = self.encoder(image, boxs[n:n+1,:])
                    deep_detections.append(self.deep_detection(boxs[n,:], confidences[n], features[0]))
                        
            # Call the tracker
            self.tracker.predict()
            self.tracker.update(deep_detections)            
            for track in self.tracker.tracks:
                if not track.is_confirmed() or track.time_since_update > 1:
                    track_id = 0
                    continue
                track_id = (int(track.track_id))
                x1, y1, x2, y2 = track.to_tlbr()
                x1 = max(min(self.image_width-1, x1), 1)
                x2 = max(min(self.image_width-1, x2), 1)
                y1 = max(min(self.image_height-1, y1), 1)
                y2 = max(min(self.image_height-1, y2), 1)   
                roi = (RegionOfInterest(x1=x1, y1=y1, x2=x2, y2=y2))
                detections_msg.objects.append(Detection(roi=roi, seg_roi=SegmentOfInterest(x=[], y=[]), id=self._new_id(), track_id=track_id,confidence=0.99, class_name="Ripe Strawberry"))
        except Exception as e:
            import sys, os
            self.currently_busy.clear()
            print("FruitCastServer error: ", e)
            exc_type, exc_obj, exc_tb = sys.exc_info()
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            print(exc_type, fname, exc_tb.tb_lineno)
            return GetDetectorResultsResponse(status=ServiceStatus(ERROR=True), results=detections_msg)
        self.currently_busy.clear()
        return GetDetectorResultsResponse(status=ServiceStatus(OKAY=True), results=detections_msg)

