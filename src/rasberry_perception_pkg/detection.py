import cv2
import numpy as np

import rospy
import tf
from SimpleDataTransport import DataSender
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header
from yeet.msg import BoundingBox, ObjectAnnotation, ObjectDescription


class DetectorClient:
    NUM_CLASSES = 80
    CLASSES = ["NA"] * NUM_CLASSES
    CLASSES[0], CLASSES[1] = "Ripe Strawberry", "Unripe Strawberry"
    COLOURS = [[255, 255, 255]] * NUM_CLASSES
    COLOURS[0], COLOURS[1] = [255, 50, 50], [50, 255, 50]

    def __init__(self, host="0.0.0.0", model_name="faster-rcnn"):
        self.host = host
        self.model_name = model_name

    def get_bounding_boxes(self, image, score=None):
        try:
            response = DataSender({"image": image, "model": self.model_name}, host=self.host)["result"]
        except Exception as e:
            rospy.loginfo("Detector 2D: Could not get detection results from host '{}':\n\t{}.".format(self.host, e))
            return None, None, None

        if "mask" in self.model_name or isinstance(response, tuple):
            bounding_boxes = response[0]
            masks = response[1]
        else:
            bounding_boxes = response
            masks = None

        # Clean bounding boxes to BoundingBox(x1, y1, x2, y2, score, class_index)
        bounding_boxes = [[BoundingBox(*list(bb) + [i]) for bb in b] for i, b in enumerate(bounding_boxes) if b.size]
        class_labels = [self.CLASSES[i] for i in range(len(bounding_boxes))]

        try:
            bounding_boxes = list(np.squeeze(np.asarray(bounding_boxes)))
        except TypeError:
            return None, None, None

        if score is not None:
            bounding_boxes = [b for b in bounding_boxes if b.score >= score]

        return bounding_boxes, masks, class_labels


class BBoxProjector:
    def __init__(self, method='std', distance_thresh=1.0, method_thresh=1.5, parent=None, child=None):
        self.method = method
        self.distance_thresh = distance_thresh * 1000
        self.method_thresh = method_thresh

        self.__transform_points = False
        if None not in (parent, child):
            self.tf_listener = tf.TransformListener()
            self.parent_frame_id = parent
            self.child_frame_id = child
            self.__transform_points = True

    @staticmethod
    def p_xyz_to_point(projection_matrix, x, y, z):
        return BBoxProjector.xyz_to_point(projection_matrix[0], projection_matrix[5], projection_matrix[2],
                                          projection_matrix[6], x, y, z)

    @staticmethod
    def xyz_to_point(fx, fy, cx, cy, x, y, z):
        return ((x - cx) * z / fx) / 1000.0, ((y - cy) * z / fy) / 1000.0, z / 1000.0

    @staticmethod
    def depth_to_pc(projection_matrix, depth_image):
        fx, fy, cx, cy = projection_matrix[0], projection_matrix[5], projection_matrix[2], projection_matrix[6]
        z_pos = (depth_image / 1000.0).flatten()
        xy_pos = np.where(np.ones_like(depth_image))
        points = np.ones((len(z_pos), 3))
        points[:, 0] = (xy_pos[0] - cx) * z_pos / fx
        points[:, 1] = ((xy_pos[1] - cy) * z_pos / fy) * -1
        points[:, 2] = z_pos * -1
        return points

    def bbox_to_object_annotation(self, depth_map, bounding_boxes, projection_matrix, header=None):
        object_annotations = []
        valid_bounding_boxes = []  # Only usable for visualisation of points on RGB image

        for bbox in bounding_boxes:
            # Get depth region of interest
            ix1, iy1, ix2, iy2 = int(bbox.x1), int(bbox.y1), int(bbox.x2), int(bbox.y2)
            depth_roi = depth_map[iy1:iy2, ix1:ix2]

            # Clean and remove nans/0/inf/>max distance
            if not np.any(depth_roi):
                continue
            valid_mask = np.logical_and(np.logical_and(depth_roi, np.isfinite(depth_roi)),
                                        depth_roi < self.distance_thresh)

            valid_positions = np.where(valid_mask)
            invalid_positions = np.where(np.logical_not(valid_mask))
            depth_roi[invalid_positions] = 0

            # Calculate valid positions for depth map
            if self.method == "std":
                std, mean = np.std(depth_roi), np.mean(depth_roi)
                valid_positions = np.where(np.logical_and((mean - self.method_thresh * std) < depth_roi, depth_roi <
                                                          (mean + self.method_thresh * std)))
            elif self.method == "rect":
                pass  # By default the valid positions are all inside the roi that are non 0 non nan
            else:
                raise ValueError('2D BBox to 3D: Method "{}" is not valid.'.format(self.method))

            z_values = depth_roi[valid_positions]

            if not z_values.size:
                continue

            # Calculate projected centroid
            z = np.mean(z_values)

            # Add y and x to valid positions to move from roi space to image space
            valid_positions = tuple(np.add(valid_positions, [[iy1], [ix1]]))
            y, x = np.mean(valid_positions, axis=1)

            # Calculate extrema regions of each strawberry axis
            # TODO(raymond): Implement point standard deviation to choose better extrema points
            if self.method == "std" or self.method == "rect":
                min_y, min_x = np.min(valid_positions, axis=1)
                max_y, max_x = np.max(valid_positions, axis=1)
                min_z, max_z = np.min(z_values), np.max(z_values)
            else:
                raise ValueError('2D BBox to 3D: Method "{}" is not valid.'.format(self.method))

            # Project all points to real space
            px, py, pz = self.p_xyz_to_point(projection_matrix, x, y, z)
            min_px, min_py, min_pz = self.p_xyz_to_point(projection_matrix, min_x, min_y, min_z)
            max_px, max_py, max_pz = self.p_xyz_to_point(projection_matrix, max_x, max_y, max_z)

            if self.__transform_points:
                try:
                    if header is None:
                        header = Header()
                        header.stamp = rospy.Time.now()
                    header.frame_id = self.child_frame_id
                    p_xyz = PointStamped(header=header, point=Point(px, py, pz))
                    p_min_xyz = PointStamped(header=header, point=Point(min_px, min_py, min_pz))
                    p_max_xyz = PointStamped(header=header, point=Point(max_px, max_py, max_pz))
                    wp_xyz = self.tf_listener.transformPoint(self.parent_frame_id, p_xyz)
                    wp_min_xyz = self.tf_listener.transformPoint(self.parent_frame_id, p_min_xyz)
                    wp_max_xyz = self.tf_listener.transformPoint(self.parent_frame_id, p_max_xyz)
                    px, py, pz = wp_xyz.point.x, wp_xyz.point.y, wp_xyz.point.z
                    min_px, min_py, min_pz = wp_min_xyz.point.x, wp_min_xyz.point.y, wp_min_xyz.point.z
                    max_px, max_py, max_pz = wp_max_xyz.point.x, wp_max_xyz.point.y, wp_max_xyz.point.z
                except (tf.Exception, tf.ExtrapolationException) as e:
                    print("\tSkipping {}, {}, {} due to exception '{}'".format(x, y, z, e))
                    continue

            description = ObjectDescription(x=px, y=py, z=pz, dl=px - min_px, dr=max_px - px, du=py - min_py,
                                            dd=max_py - py, df=pz - min_pz, db=max_pz - pz, class_id=bbox.class_id)
            object_annotations.append(ObjectAnnotation(bounding_box=bbox, description=description))

            # For visualisation
            valid_bounding_boxes.append([bbox, valid_positions])

        return object_annotations, valid_bounding_boxes

    @staticmethod
    def visualise_detection(rgb_image, b_boxes, class_names, class_colours):
        # Visualise detection
        detection_canvas = rgb_image
        overlay_canvas = np.zeros_like(detection_canvas)
        font, font_scale, thickness, alpha = cv2.FONT_HERSHEY_DUPLEX, 0.55, 1, 0.5

        # Add depth map points to overlay as class colour
        for bbox, valid_positions in b_boxes:
            overlay_canvas[valid_positions] = class_colours[bbox.class_id]

        # Create blend of points map and image
        blend_idx = np.where(overlay_canvas)
        blended = detection_canvas
        blended[blend_idx] = (detection_canvas[blend_idx] * (1.0 - alpha) + overlay_canvas[blend_idx]
                              * alpha).astype(detection_canvas[blend_idx].dtype)

        # Add text to blended image and bbox overlay
        for bbox, valid_positions in b_boxes:
            text = "{} {:5.2f}%".format(class_names[bbox.class_id], bbox.score * 100)
            size = cv2.getTextSize(text, font, font_scale, thickness)
            cv2.putText(blended, text, (int(bbox.x1), int(bbox.y1 - size[1])), font, font_scale,
                        class_colours[bbox.class_id], thickness + 1)
            cv2.putText(blended, text, (int(bbox.x1), int(bbox.y1 - size[1])), font, font_scale, (255, 255, 255),
                        thickness)
            cv2.rectangle(blended, (int(bbox.x1), int(bbox.y1)), (int(bbox.x2), int(bbox.y2)),
                          class_colours[bbox.class_id], thickness=thickness)

        return blended, overlay_canvas
