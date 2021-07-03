#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com
from threading import Event

import ros_numpy
import numpy as np
from rasberry_perception.interfaces.default import BaseDetectionServer
from rasberry_perception.interfaces.registry import DETECTION_REGISTRY
from rasberry_perception.msg import Detections, Detection, ServiceStatus, RegionOfInterest, SegmentOfInterest
# from rasberry_perception.visualisation import GenericMask
from rasberry_perception.srv import GetDetectorResultsResponse, GetDetectorResultsRequest
from rasberry_perception.utility import function_timer

@DETECTION_REGISTRY.register_detection_backend("yolov4deepsort")
class YoloV4DeepsortServer(BaseDetectionServer):
    def __init__(self, weightPath="/yolov4_sb_best.weights", configPath="/yolov4_sb.cfg", metaPath="/voc_sb.data", deepsort_modelPath="/mars_sb_14.pb", image_size=640, nms_conf_thresh=0.4, nms_iou_thresh=0.5,max_cosine_distance = 0.6, nn_budget = 50, nms_max_overlap = 1.0):
        try:
            import darknet            
            from deep_sort.tracker import Tracker
            from deep_sort import nn_matching
            from deep_sort.tools import generate_detections as gdet
            from deep_sort import preprocessing
            from deep_sort.detection import Detection as deep_detection
            import os
        except ImportError:
            raise
        self.image_size = image_size
        project_dir = os.path.dirname(__file__)
        configPath = os.path.join(project_dir, configPath)
        weightPath = os.path.join(project_dir, weightPath)
        metaPath = os.path.join(project_dir, metaPath)
        deepsort_modelPath = os.path.join(project_dir, deepsort_modelPath)
        self.network, self.class_names, self.class_colors = darknet.load_network(configPath, metaPath, weightPath, batch_size=1)
        self.nms_max_overlap = nms_max_overlap
       
        #DarknetImage
        self.darknet = darknet
        ww = 640
        hh = 480
        self.darknet_image = darknet.make_image(ww, hh, 3)
        
        # deep_sort
        self.preprocessing = preprocessing
        self.deep_detection = deep_detection
        self.encoder = gdet.create_box_encoder(deepsort_modelPath, batch_size=1)
        metric = nn_matching.NearestNeighborDistanceMetric("cosine", max_cosine_distance, nn_budget)
        self.tracker = Tracker(metric)     
        BaseDetectionServer.__init__(self)

    @staticmethod
    def citation_notice():
        return "YoloV4 Inference and Feature Extractor by Ya Xiong(Bill)\n" \
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
            import cv2
        except ImportError:
            raise

        if self.currently_busy.is_set():
            return GetDetectorResultsResponse(status=ServiceStatus(BUSY=True))
        self.currently_busy.set()
        detections_msg = Detections()
        try:
            frame = ros_numpy.numpify(request.image)
            original_shape = frame.shape
            frame = cv2.resize(frame, (self.image_size, int(self.image_size*0.75)))
            darknet.copy_image_from_bytes(self.darknet_image, frame.tobytes())   
            detections_yolo = darknet.detect_image(self.network, self.class_names, self.darknet_image, thresh=0.7)
            boxs = []
            confidences = []
            class_name= []
            for detection in detections_yolo:   
                print(detection)
                if detection[0] != "ripe":  # only track ripe berry whose id is 0    
                    x1, y1, x2, y2 = self._convertBack(detection[2][0], \
                    detection[2][1], \
                    detection[2][2], \
                    detection[2][3])
                    x1 = x1*original_shape[1]/frame.shape[1]
                    x2 = x2*original_shape[1]/frame.shape[1]
                    y1 = y1*original_shape[0]/frame.shape[0]
                    y2 = y2*original_shape[0]/frame.shape[0]
                    x1 = max(min(original_shape[1]-1, x1), 1)
                    x2 = max(min(original_shape[1]-1, x2), 1)
                    y1 = max(min(original_shape[0]-1, y1), 1)
                    y2 = max(min(original_shape[0]-1, y2), 1)
                    detections_msg.objects.append(Detection(roi=RegionOfInterest(x1=x1, y1=y1, x2=x2, y2=y2), seg_roi=SegmentOfInterest(x=[], y=[]), id=self._new_id(), track_id=-1, confidence=float(detection[1])/100, class_name="unripe"))                        
                    continue
                confidences.append(float(detection[1])/100)
                class_name.append(detection[0])
                bounds = detection[2]   
                xCoord = int(bounds[0] - bounds[2] / 2)
                yCoord = int(bounds[1] - bounds[3] / 2)
                boxs.append([xCoord, yCoord, int(bounds[2]), int(bounds[3])])
            features = self.encoder(frame, boxs)
            detections = [self.deep_detection(bbox, confidence, feature) for bbox, confidence, feature in
                        zip(boxs, confidences, features)]
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
                x1 = x1*original_shape[1]/frame.shape[1] 
                x2 = x2*original_shape[1]/frame.shape[1]  
                y1 = y1*original_shape[0]/frame.shape[0]  
                y2 = y2*original_shape[0]/frame.shape[0]
                x1 = max(min(original_shape[1]-1, x1), 1)
                x2 = max(min(original_shape[1]-1, x2), 1)
                y1 = max(min(original_shape[0]-1, y1), 1)
                y2 = max(min(original_shape[0]-1, y2), 1)                     
                roi = (RegionOfInterest(x1=x1, y1=y1, x2=x2, y2=y2))
                detections_msg.objects.append(Detection(roi=roi, seg_roi=SegmentOfInterest(x=[], y=[]), id=self._new_id(), track_id=track_id,confidence=0.99, class_name="Ripe Strawberry"))
            self.currently_busy.clear()
        except Exception as e:
            print("FruitCastServer error: ", e)
            return GetDetectorResultsResponse(status=ServiceStatus(ERROR=True), results=detections_msg)
        return GetDetectorResultsResponse(status=ServiceStatus(OKAY=True), results=detections_msg)

    def _convertBack(self, x, y, w, h):
        xmin = int(round(x - (w / 2)))
        xmax = int(round(x + (w / 2)))
        ymin = int(round(y - (h / 2)))
        ymax = int(round(y + (h / 2)))
        return xmin, ymin, xmax, ymax