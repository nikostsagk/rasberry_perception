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
    def __init__(self, config_path, service_name, image_height=480, image_width=640, image_hz=30, deepsort_modelPath="/mars_sb_14.pb",max_cosine_distance = 0.6, nn_budget = 200, nms_max_overlap = 1.0):    
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
            boxs = []
            confidences = []
            class_name= []
            self.mot.step(image)
            for detection in self.mot.detections:
                if detection[1] != "ripe":  # only track ripe berry whose class is "ripe"        
                    x1, y1, x2, y2 = detection[0][0], \
                    detection[0][1], \
                    detection[0][2], \
                    detection[0][3]
                    detections_msg.objects.append(Detection(roi=RegionOfInterest(x1=x1, y1=y1, x2=x2, y2=y2), seg_roi=SegmentOfInterest(x=[], y=[]), id=self._new_id(), track_id=-1, confidence=detection[2], class_name="unripe"))                        
                    continue
                confidences.append(detection[2])
                class_name.append(detection[1])
                bounds = detection[0]
                h = bounds[3] - bounds[1]
                w = bounds[2] - bounds[0]
                boxs.append([int(bounds[0]), int(bounds[1]), int(w), int(h)])
            features = self.encoder(image, boxs)
            detections = [self.deep_detection(bbox, confidence, feature) for bbox, confidence, feature in zip(boxs, confidences, features)]
            # Run non-maxima suppression.
            boxes = np.array([d.tlwh for d in detections])
            scores = np.array([d.confidence for d in detections])
            indices = self.preprocessing.non_max_suppression(boxes, self.nms_max_overlap, scores)
            detections = [detections[i] for i in indices]
            # Call the tracker
            self.tracker.predict()
            self.tracker.update(detections)            
            for track in self.tracker.tracks:
                if not track.is_confirmed() or track.time_since_update > 1:
                    track_id = 0
                    continue
                track_id = (int(track.track_id))
                bbox = track.to_tlbr()
                x1, y1, x2, y2 = bbox[0],bbox[1],bbox[2],bbox[3]  
                x1 = max(min(self.image_width-1, x1), 1)
                x2 = max(min(self.image_width-1, x2), 1)
                y1 = max(min(self.image_height-1, y1), 1)
                y2 = max(min(self.image_height-1, y2), 1)   
                roi = (RegionOfInterest(x1=x1, y1=y1, x2=x2, y2=y2))
                detections_msg.objects.append(Detection(roi=roi, seg_roi=SegmentOfInterest(x=[], y=[]), id=self._new_id(), track_id=track_id,confidence=0.99, class_name="Ripe Strawberry"))
        except Exception as e:
            self.currently_busy.clear()
            print("FruitCastServer error: ", e)
            return GetDetectorResultsResponse(status=ServiceStatus(ERROR=True), results=detections_msg)
        self.currently_busy.clear()
        return GetDetectorResultsResponse(status=ServiceStatus(OKAY=True), results=detections_msg)

