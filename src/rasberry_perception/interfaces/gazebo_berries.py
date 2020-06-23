#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com
from threading import Event

import ros_numpy
from rasberry_perception.msg import Detections, ServiceStatus, RegionOfInterest, SegmentOfInterest, Detection

from rasberry_perception.interfaces.default import BaseDetectionServer
from rasberry_perception.interfaces.registry import DETECTION_REGISTRY
from rasberry_perception.utility import function_timer
from rasberry_perception.srv import GetDetectorResultsResponse, GetDetectorResultsRequest


class _unknown_class:
    def __init__(self):
        pass

    def __getitem__(self, item):
        return "class {}".format(item)


@DETECTION_REGISTRY.register_detection_backend("gazebo_berries")
class GazeboRenderedBerriesServer(BaseDetectionServer):
    def __init__(self, keyword="strawberry", ref_frame='camera1d435e_camera'):
        try:
            from geometry_msgs.msg import PoseStamped
            import rospy
            import tf2_geometry_msgs
            import rospy
            import tf2_ros
            import tf2_geometry_msgs
            from gazebo_msgs.msg import LinkStates
            from geometry_msgs.msg import PoseStamped
            pass
        except ImportError:
            raise

        self.currently_busy = Event()
        self.classes = _unknown_class()

        self.keyword    = keyword
        self.ref_frame  = ref_frame
        self.obj_poses  = []
        self.obj_ids    = [] 
        self.states_sub = rospy.Subscriber("/gazebo/link_states", 
                                           LinkStates, self.states_cb)
        self.tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Base class must be called at the end due to self.service_server.spin()
        BaseDetectionServer.__init__(self)

    def states_cb(self, links):
        # Get objects of interest
        self.obj_poses = []
        self.obj_ids   = [] 
        for idx, name in enumerate(links.name):
          if self.keyword in name:
            pose = PoseStamped()
            pose.pose = links.pose[idx]
            self.obj_poses.append(pose)
            self.obj_ids.append(idx)

    @function_timer.interval_logger(interval=10)
    def get_detector_results(self, request):
        """

        Args:
            request (GetDetectorResultsRequest):

        Returns:
            GetDetectorResultsResponse
        """
        if self.currently_busy.is_set():
            return GetDetectorResultsResponse(status=ServiceStatus(BUSY=True))
        self.currently_busy.set()

        detections = Detections()

        try:
            rois  = []
            poses = []
            for pose in self.obj_poses:
                transform = self.tf_buffer.lookup_transform(self.ref_frame, 'odom', rospy.Time())
                pose = tf2_geometry_msgs.do_transform_pose(pose, transform).pose
                x1 = pose.position.x - 0.03
                x2 = pose.position.x + 0.03
                y1 = pose.position.y - 0.03
                y2 = pose.position.y + 0.03
                z1 = pose.position.z - 0.03
                z2 = pose.position.z + 0.03
                roi = RegionOfInterest(x1=x1, y1=y1, z1=z1, x2=x2, y2=y2, z2=z2)
                rois.append(roi)
                poses.append(pose)

            self.currently_busy.clear()

            # Fill in the detections message
            for roi, pose, track_id in zip(rois, poses, self.obj_ids):
                seg_roi = SegmentOfInterest(x=[], y=[])
                score = 1.0
                detections.objects.append(Detection(roi=roi, seg_roi=seg_roi, pose=pose,
                                                    id=self._new_id(), track_id=track_id,
                                                    confidence=score, class_name=self.classes[0]))

            return GetDetectorResultsResponse(status=ServiceStatus(OKAY=True), results=detections)
        except Exception as e:
            print("GazeboRenderedBerriesServer error: ", e)
            return GetDetectorResultsResponse(status=ServiceStatus(ERROR=True), results=detections)
